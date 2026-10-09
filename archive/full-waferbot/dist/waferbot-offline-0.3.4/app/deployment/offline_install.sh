#!/usr/bin/env bash
# Install waferbot from an offline bundle with no network access at all.
#
# Every pip invocation uses --no-index --find-links <bundle>/wheels: this script
# never upgrades pip, never contacts PyPI, and never resolves anything remotely.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BUNDLE="$(cd -- "$SCRIPT_DIR/.." && pwd)"

BUNDLE_DIR="$DEFAULT_BUNDLE"
VENV_DIR="${WAFERBOT_VENV:-$HOME/.venvs/waferbot}"
PYTHON_BIN="${WAFERBOT_PYTHON:-python3}"
SKIP_HASH_CHECK=0

usage() {
  cat <<'USAGE'
Usage: offline_install.sh [options]

  --bundle DIR   offline bundle directory (default: the bundle containing this script)
  --venv DIR     virtual environment location (default: ~/.venvs/waferbot)
  --python BIN   interpreter used to create the venv (default: python3)
  --skip-hash-check  do not verify file hashes against manifest.json
  -h, --help     show this help
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --bundle) BUNDLE_DIR="${2:?--bundle needs a directory}"; shift 2 ;;
    --venv) VENV_DIR="${2:?--venv needs a directory}"; shift 2 ;;
    --python) PYTHON_BIN="${2:?--python needs an interpreter}"; shift 2 ;;
    --skip-hash-check) SKIP_HASH_CHECK=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ ! -d "$BUNDLE_DIR/wheels" ]; then
  echo "bundle has no wheels directory: $BUNDLE_DIR/wheels" >&2
  exit 2
fi
if ! compgen -G "$BUNDLE_DIR/wheels/waferbot-*.whl" > /dev/null; then
  echo "bundle is missing the waferbot wheel" >&2
  exit 2
fi

if [ "$SKIP_HASH_CHECK" -eq 0 ]; then
  if [ ! -f "$BUNDLE_DIR/manifest.json" ]; then
    echo "manifest.json missing; re-run with --skip-hash-check to continue" >&2
    exit 2
  fi
  echo "Verifying bundle hashes (no network access is used)"
  python3 - "$BUNDLE_DIR" <<'PY'
import hashlib, json, pathlib, sys

bundle = pathlib.Path(sys.argv[1])
manifest = json.loads((bundle / "manifest.json").read_text())
checked = 0
for entry in manifest["files"]:
    path = bundle / entry["path"]
    if not path.exists():
        raise SystemExit(f"missing bundle file: {entry['path']}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != entry["sha256"]:
        raise SystemExit(f"hash mismatch for {entry['path']}")
    checked += 1
required = {"waferbot", "smbus2"}
present = {name.split("-")[0] for name in (p.name for p in (bundle / "wheels").glob("*.whl"))}
missing = required - present
if missing:
    raise SystemExit(f"bundle is missing required wheels: {sorted(missing)}")
print(f"verified {checked} files and required wheels {sorted(required)}")
PY
fi

echo "Creating virtual environment at $VENV_DIR"
"$PYTHON_BIN" -m venv "$VENV_DIR"

echo "Installing waferbot offline (--no-index, wheels only)"
PYTHONNOUSERSITE=1 PIP_NO_INDEX=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
  "$VENV_DIR/bin/python" -m pip install --upgrade --force-reinstall \
  --no-index --find-links "$BUNDLE_DIR/wheels" \
  --disable-pip-version-check "waferbot[hardware]"

cat <<EOF

Offline installation complete.

  interpreter : $VENV_DIR/bin/python
  cli         : $VENV_DIR/bin/waferbot

Use it with an explicit shell path (no activation required):
  $VENV_DIR/bin/waferbot --version

Or activate the environment first:
  . "$VENV_DIR/bin/activate"
  waferbot --version

Nothing starts automatically. Read-only hardware check next:
  bash deployment/verify_hardware.sh --venv "$VENV_DIR"
EOF
