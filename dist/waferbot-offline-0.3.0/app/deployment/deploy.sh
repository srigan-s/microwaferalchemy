#!/usr/bin/env bash
# Deploy from a development machine (for example macOS) to a Raspberry Pi over SSH.
#
# This script copies files and runs the installer. It never erases or formats
# storage: flashing the SD card is a separate, manual step with Raspberry Pi
# Imager (see README.md).
#
# The remote layout is the bundle layout:
#
#   <remote-dir>/deployment/install_pi.sh   <- run from <remote-dir>
#   <remote-dir>/wheels/                    <- offline wheels
#   <remote-dir>/app/                       <- allowlisted application files
set -euo pipefail

HOST="${WAFERBOT_HOST:-waferbot.local}"
USER_NAME="${WAFERBOT_USER:-pi}"
PORT="${WAFERBOT_SSH_PORT:-22}"
REMOTE_DIR="${WAFERBOT_REMOTE_DIR:-waferbot-bundle}"
LOCAL_BUNDLE=""
CUSTOM_VENV=""
BUILD=0
DRY_RUN=0

usage() {
  cat <<'USAGE'
Usage: deploy.sh [options]

  --host HOST        target hostname or IP (default: waferbot.local)
  --user USER        ssh user (default: pi)
  --port N           ssh port (default: 22)
  --remote-dir DIR   directory on the Pi for the bundle (default: waferbot-bundle)
  --bundle DIR       local offline bundle (default: newest dist/waferbot-offline-*)
  --build            build the offline bundle first
  --venv DIR         virtual environment on the Pi (default: $HOME/.venvs/waferbot,
                     expanded on the Pi)
  --dry-run          print the commands (run.sh prints the remote commands too)
  -h, --help         show this help
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --host) HOST="${2:?--host needs a value}"; shift 2 ;;
    --user) USER_NAME="${2:?--user needs a value}"; shift 2 ;;
    --port) PORT="${2:?--port needs a value}"; shift 2 ;;
    --remote-dir) REMOTE_DIR="${2:?--remote-dir needs a value}"; shift 2 ;;
    --bundle) LOCAL_BUNDLE="${2:?--bundle needs a directory}"; shift 2 ;;
    --build) BUILD=1; shift ;;
    --venv) CUSTOM_VENV="${2:?--venv needs a value}"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

validate_token() {
  local name="$1" value="$2"
  if [ -z "$value" ]; then
    echo "$name must not be empty" >&2
    exit 2
  fi
  case "$value" in
    -*) echo "$name must not start with '-'" >&2; exit 2 ;;
  esac
  if printf '%s' "$value" | grep -qE "[[:space:];&|\$()<>\"'\\\\]"; then
    echo "$name contains characters that are unsafe for remote commands: $value" >&2
    exit 2
  fi
}

validate_token "host" "$HOST"
validate_token "user" "$USER_NAME"
validate_token "remote-dir" "$REMOTE_DIR"
case "$PORT" in
  ''|*[!0-9]*) echo "port must be numeric" >&2; exit 2 ;;
esac
if [ -n "$CUSTOM_VENV" ]; then
  # A custom venv path is single-quoted into the remote command, so it must not
  # contain quotes, whitespace, or shell metacharacters.
  validate_token "venv" "$CUSTOM_VENV"
  REMOTE_VENV="'$CUSTOM_VENV'"
else
  # Deliberate remote expansion: the default venv lives under the Pi user's HOME.
  REMOTE_VENV='$HOME/.venvs/waferbot'
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [ "$BUILD" -eq 1 ]; then
  echo "Building offline bundle"
  python3 "$REPO_DIR/scripts/build_offline_bundle.py" --output-dir "$REPO_DIR/dist"
fi

if [ -z "$LOCAL_BUNDLE" ]; then
  LOCAL_BUNDLE="$(find "$REPO_DIR/dist" -maxdepth 1 -type d -name 'waferbot-offline-*' 2>/dev/null | sort | tail -n 1 || true)"
fi
if [ -z "$LOCAL_BUNDLE" ] || [ ! -d "$LOCAL_BUNDLE" ]; then
  echo "no offline bundle found; run scripts/build_offline_bundle.py or pass --bundle" >&2
  exit 2
fi
if [ ! -f "$LOCAL_BUNDLE/deployment/install_pi.sh" ]; then
  echo "bundle is missing deployment/install_pi.sh: $LOCAL_BUNDLE" >&2
  exit 2
fi

TARGET="$USER_NAME@$HOST"

REMOTE_MKDIR="mkdir -p -- '$REMOTE_DIR'"
REMOTE_INSTALL="cd '$REMOTE_DIR' && bash deployment/install_pi.sh --bundle . --venv $REMOTE_VENV --yes"
REMOTE_VERIFY="cd '$REMOTE_DIR' && bash deployment/verify_hardware.sh --venv $REMOTE_VENV"

echo "Deploying $(basename "$LOCAL_BUNDLE") to $TARGET:$REMOTE_DIR"

run() {
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '+ %s\n' "$*"
  else
    "$@"
  fi
}

# Stock macOS rsync has no --info=progress2; keep the flags portable.
run ssh -p "$PORT" "$TARGET" "$REMOTE_MKDIR"
run rsync -az --partial -e "ssh -p $PORT" \
  "$LOCAL_BUNDLE/" "$TARGET:$REMOTE_DIR/"

if [ "$DRY_RUN" -eq 1 ]; then
  printf 'REMOTE_CMD: %s\n' "$REMOTE_INSTALL"
  printf 'REMOTE_CMD: %s\n' "$REMOTE_VERIFY"
else
  run ssh -p "$PORT" "$TARGET" "$REMOTE_INSTALL"
  run ssh -p "$PORT" "$TARGET" "$REMOTE_VERIFY"
fi

cat <<EOF

Deployment finished (or printed, with --dry-run).

Next, on the Pi:
  ssh $USER_NAME@$HOST
  ~/.venvs/waferbot/bin/waferbot sensors --physical --config ~/.config/waferbot/robot.example.json

Flashing the SD card is a manual step and is not performed here; see README.md
("Flashing the SD card") for the Raspberry Pi Imager workflow.
EOF
