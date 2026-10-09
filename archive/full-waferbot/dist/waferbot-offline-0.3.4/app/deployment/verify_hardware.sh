#!/usr/bin/env bash
# Read-only hardware verification: I2C bus, controller address, sensor frames.
#
# The sensor read uses the read-only diagnostic lifecycle
# (`waferbot sensors`, which never commands the motors, not even on exit), and
# the requested bus/address are propagated into the CLI through a temporary
# config. No motor command is issued by this script.
set -euo pipefail

VENV_DIR="${WAFERBOT_VENV:-$HOME/.venvs/waferbot}"
EXPECTED_ADDRESS="${WAFERBOT_I2C_ADDRESS:-2b}"
BUS="${WAFERBOT_I2C_BUS:-1}"
SAMPLES="${WAFERBOT_SAMPLES:-5}"
CONFIG="${WAFERBOT_CONFIG:-}"
STATUS=0

usage() {
  cat <<'USAGE'
Usage: verify_hardware.sh [options]

  --venv DIR     virtual environment with waferbot installed
  --bus N        I2C bus number (default: 1)
  --address HEX  expected controller address (default: 2b)
  --samples N    sensor samples to read (default: 5)
  --config PATH  robot config to use for the sensor read
  -h, --help     show this help

This script issues no motor commands.
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --venv) VENV_DIR="${2:?--venv needs a directory}"; shift 2 ;;
    --bus) BUS="${2:?--bus needs a number}"; shift 2 ;;
    --address) EXPECTED_ADDRESS="${2:?--address needs a hex value}"; shift 2 ;;
    --samples) SAMPLES="${2:?--samples needs a number}"; shift 2 ;;
    --config) CONFIG="${2:?--config needs a path}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

CLI="$VENV_DIR/bin/waferbot"

echo "== I2C devices on bus $BUS"
if command -v i2cdetect >/dev/null 2>&1; then
  i2cdetect -y "$BUS" || STATUS=1
  if i2cdetect -y "$BUS" | grep -qi -- "$EXPECTED_ADDRESS"; then
    echo "controller 0x$EXPECTED_ADDRESS detected"
  else
    echo "controller 0x$EXPECTED_ADDRESS NOT detected on bus $BUS" >&2
    STATUS=1
  fi
else
  echo "i2cdetect not installed (sudo apt-get install i2c-tools)" >&2
  STATUS=1
fi

TMP_CONFIG=""
if [ -z "$CONFIG" ]; then
  # Propagate --bus/--address into the CLI so the sensor read uses the same bus.
  TMP_CONFIG="$(mktemp -t waferbot-verify-XXXXXX.json)"
  python3 - "$TMP_CONFIG" "$BUS" "$EXPECTED_ADDRESS" <<'PY'
import json, sys

path, bus, address = sys.argv[1], int(sys.argv[2]), int(sys.argv[3], 16)
config = {
    "motor": {"bus": bus, "address": address},
    "sensor": {"bus": bus, "address": address},
    "safety": {},
}
with open(path, "w", encoding="utf-8") as handle:
    json.dump(config, handle, indent=2)
PY
  CONFIG="$TMP_CONFIG"
fi
cleanup() {
  if [ -n "$TMP_CONFIG" ] && [ -f "$TMP_CONFIG" ]; then
    rm -f -- "$TMP_CONFIG"
  fi
}
trap cleanup EXIT

echo
echo "== Line sensor frames (read-only, no motor commands)"
if [ -x "$CLI" ]; then
  if "$CLI" sensors --physical --config "$CONFIG" --samples "$SAMPLES"; then
    echo "sensor read OK"
  else
    echo "sensor read FAILED" >&2
    STATUS=1
  fi
else
  echo "waferbot CLI not found at $CLI (run deployment/install_pi.sh first)" >&2
  STATUS=1
fi

cat <<EOF

Next steps (each is manual, prompted, and needs a rendered/verified config):
  1. Wheels OFF the ground:   $CLI motor-test --physical --config ~/.config/waferbot/robot.first-run.json --speed 5
  2. Confirm sensor mapping:  $CLI calibrate sensors --physical --config ~/.config/waferbot/robot.first-run.json
  3. Mark motor and sensor verified=true in the robot config after those pass.
  4. Straight-line follow:    $CLI follow --physical --config ~/.config/waferbot/robot.first-run.json \\
                                --nav-config ~/waferbot-test/app/config/nav.first-run.json --edge black-left --duration 3
  5. Edge switch:             $CLI switch --physical --config ~/.config/waferbot/robot.json \\
                                --nav-config ~/.config/waferbot/nav.json \\
                                --from-edge black-left --to-edge black-right \\
                                --location B2 --route-id manual --authorize
  6. A+ -> B- -> D-:          $CLI execute --physical --config ~/.config/waferbot/robot.json \\
                                --nav-config ~/.config/waferbot/nav.json \\
                                --map <measured-map.json> --start A+ --goal D- --via B- \\
                                --localizer manual

Reminder: 'waferbot' cannot cut motor power. Keep a physical power switch within
reach and stop the chassis with it if anything behaves unexpectedly.
EOF

exit "$STATUS"
