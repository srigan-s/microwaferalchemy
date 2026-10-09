#!/usr/bin/env bash
# Install waferbot on a Raspberry Pi 5 running Raspberry Pi OS 64-bit.
#
# This script never starts the robot and never installs an autostart service:
# motion is only possible from an interactive, explicitly confirmed command.
#
# It may install system packages with apt (python3-venv, python3-dev, i2c-tools,
# libi2c-dev) and enable the I2C interface, matching the Raspberry Pi
# documentation (Configuration > Hardware communication > I2C).
set -euo pipefail

VENV_DIR="${WAFERBOT_VENV:-$HOME/.venvs/waferbot}"
LOG_DIR="${WAFERBOT_LOG_DIR:-$HOME/waferbot-logs}"
CONFIG_DIR="${WAFERBOT_CONFIG_DIR:-$HOME/.config/waferbot}"
BUNDLE_DIR=""
WHEEL_PATH=""
SOURCE_DIR=""
ENABLE_I2C=1
INSTALL_APT=1
ASSUME_YES=0

usage() {
  cat <<'USAGE'
Usage: install_pi.sh [options]

  --bundle DIR      install from an offline bundle built by scripts/build_offline_bundle.py
  --wheel PATH      install a single prebuilt waferbot wheel
  --source DIR      install from a source checkout (default: this repository)
  --venv DIR        virtual environment location (default: ~/.venvs/waferbot)
  --log-dir DIR     directory for telemetry logs (default: ~/waferbot-logs)
  --config-dir DIR  directory for robot/nav config copies (default: ~/.config/waferbot)
  --no-apt          skip apt package installation
  --no-i2c          skip enabling the I2C interface
  --yes             do not prompt before apt/raspi-config changes
  -h, --help        show this help
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --bundle) BUNDLE_DIR="${2:?--bundle needs a directory}"; shift 2 ;;
    --wheel) WHEEL_PATH="${2:?--wheel needs a path}"; shift 2 ;;
    --source) SOURCE_DIR="${2:?--source needs a directory}"; shift 2 ;;
    --venv) VENV_DIR="${2:?--venv needs a directory}"; shift 2 ;;
    --log-dir) LOG_DIR="${2:?--log-dir needs a directory}"; shift 2 ;;
    --config-dir) CONFIG_DIR="${2:?--config-dir needs a directory}"; shift 2 ;;
    --no-apt) INSTALL_APT=0; shift ;;
    --no-i2c) ENABLE_I2C=0; shift ;;
    --yes) ASSUME_YES=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [ -z "$SOURCE_DIR" ] && [ -z "$BUNDLE_DIR" ] && [ -z "$WHEEL_PATH" ]; then
  SOURCE_DIR="$REPO_DIR"
fi

confirm() {
  local prompt="$1"
  if [ "$ASSUME_YES" -eq 1 ]; then
    return 0
  fi
  read -r -p "$prompt [y/N] " reply
  case "$reply" in
    y|Y|yes|YES) return 0 ;;
    *) echo "skipped: $prompt" ; return 1 ;;
  esac
}

echo "Python:      $(python3 --version 2>/dev/null || echo 'python3 missing')"
echo "Machine:     $(uname -s) $(uname -m)"
echo "venv:        $VENV_DIR"
echo "logs:        $LOG_DIR"

if [ "$(uname -s)" != "Linux" ]; then
  echo "warning: this installer targets Raspberry Pi OS (Linux); detected $(uname -s)" >&2
fi
case "$(uname -m)" in
  aarch64|arm64) ;;
  *) echo "warning: 64-bit OS expected (aarch64); detected $(uname -m)" >&2 ;;
esac

if [ "$INSTALL_APT" -eq 1 ]; then
  if command -v apt-get >/dev/null 2>&1; then
    if confirm "Install system packages with apt (python3-venv, python3-dev, i2c-tools, libi2c-dev)?"; then
      sudo apt-get update
      sudo apt-get install -y python3-venv python3-dev i2c-tools libi2c-dev
    fi
  else
    echo "note: apt-get not found; install python3-venv, i2c-tools manually" >&2
  fi
fi

if [ "$ENABLE_I2C" -eq 1 ]; then
  if command -v raspi-config >/dev/null 2>&1; then
    if confirm "Enable the I2C interface (raspi-config nonint do_i2c 0)?"; then
      sudo raspi-config nonint do_i2c 0 || {
        echo "note: enable I2C manually: sudo raspi-config > 3 Interface Options > I5 I2C" >&2
      }
    fi
  else
    echo "note: raspi-config not found; enable I2C manually (Control Centre > Interfaces > I2C)" >&2
  fi
  if [ ! -e /dev/i2c-1 ]; then
    echo "note: /dev/i2c-1 is not present yet; a reboot is usually required" >&2
  fi
fi

if [ -n "$BUNDLE_DIR" ]; then
  if [ ! -d "$BUNDLE_DIR" ]; then
    echo "bundle directory not found: $BUNDLE_DIR" >&2
    exit 2
  fi
  # Offline path: no pip upgrade, no index, no network at all.
  echo "Installing from offline bundle $BUNDLE_DIR (no network access)"
  # Share manifest verification and offline upgrade semantics with the standalone
  # installer, including when a previous waferbot release is already installed.
  bash "$SCRIPT_DIR/offline_install.sh" --bundle "$BUNDLE_DIR" --venv "$VENV_DIR"
elif [ -n "$WHEEL_PATH" ]; then
  python3 -m venv "$VENV_DIR"
  "$VENV_DIR/bin/python" -m pip install "$WHEEL_PATH"
else
  # Source install: this is the only path that may reach the network.
  python3 -m venv "$VENV_DIR"
  "$VENV_DIR/bin/python" -m pip install --upgrade pip
  echo "Installing from source at $SOURCE_DIR"
  ( cd "$SOURCE_DIR" && "$VENV_DIR/bin/python" -m pip install ".[hardware]" )
fi

mkdir -p "$LOG_DIR" "$CONFIG_DIR"
if [ ! -w "$LOG_DIR" ] || [ ! -w "$CONFIG_DIR" ]; then
  echo "warning: $LOG_DIR or $CONFIG_DIR is not writable by $(id -un)" >&2
fi

copy_template() {
  local name="$1"
  local target="$CONFIG_DIR/$name"
  if [ -f "$target" ]; then
    echo "keeping existing config: $target"
    return 0
  fi
  for candidate in "$REPO_DIR/config/$name" "$SOURCE_DIR/config/$name" "$BUNDLE_DIR/app/config/$name"; do
    if [ -n "$candidate" ] && [ -f "$candidate" ]; then
      cp -- "$candidate" "$target"
      echo "wrote $target"
      return 0
    fi
  done
  echo "note: $name template not found; copy it from the repository config/ directory" >&2
}

copy_template robot.example.json
copy_template nav.example.json

# I2C access for the current user.
if getent group i2c >/dev/null 2>&1; then
  if id -nG "$(id -un)" | tr ' ' '\n' | grep -qx i2c; then
    echo "user $(id -un) is already in the i2c group"
  else
    echo "adding $(id -un) to the i2c group (log out and back in, or reboot, to apply)"
    sudo usermod -aG i2c "$(id -un)" || {
      echo "note: could not add the user to the i2c group; adjust permissions manually" >&2
    }
  fi
else
  echo "note: no i2c group on this system; check /dev/i2c-* permissions manually" >&2
fi
if [ ! -r /dev/i2c-1 ] || [ ! -w /dev/i2c-1 ]; then
  echo "note: /dev/i2c-1 is not readable+writable yet; enable I2C and reboot" >&2
fi

cat <<EOF

Installation complete.

  interpreter : $VENV_DIR/bin/python
  cli         : $VENV_DIR/bin/waferbot
  logs        : $LOG_DIR
  config      : $CONFIG_DIR

Activate with:   . "$VENV_DIR/bin/activate"
Or call it directly:  $VENV_DIR/bin/waferbot --version

Nothing was installed to start on boot on purpose: this chassis must never move
without an operator present. First checks (no motion):

  $VENV_DIR/bin/waferbot sensors --physical --config "$CONFIG_DIR/robot.example.json" --samples 5

Then, with the wheels off the ground and a second person watching:

  $VENV_DIR/bin/waferbot motor-test --physical --config "$CONFIG_DIR/robot.example.json" --speed 5

Read README.md ("First hardware session") before driving on the floor.
EOF
