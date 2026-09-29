"""Command-shim integration tests for deployment scripts.

The ssh/rsync shims execute the *generated remote commands* in a temporary fake
Pi layout, proving the paths and arguments deploy.sh produces without opening a
connection, running apt, or touching hardware.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent

SSH_SHIM = """#!/usr/bin/env bash
# ssh shim: drop -p/port and the target, then run the remote command locally.
set -euo pipefail
args=("$@")
while [ "${args[0]:-}" = "-p" ]; do
  args=("${args[@]:2}")
done
target="${args[0]}"
command="${args[1]}"
printf '%s\\n' "$command" >> "$SHIM_LOG/ssh-commands.log"
cd "$SHIM_FAKE_HOME"
bash -c "$command"
"""

RSYNC_SHIM = """#!/usr/bin/env bash
# rsync shim: copy SRC into the local directory named by user@host:PATH.
set -euo pipefail
positional=()
skip_next=0
for arg in "$@"; do
  if [ "$skip_next" = "1" ]; then
    skip_next=0
    continue
  fi
  case "$arg" in
    -e) skip_next=1 ;;
    -*) ;;
    *ssh*) ;;
    *) positional+=("$arg") ;;
  esac
done
src="${positional[0]}"
last="${positional[1]}"
dest="${last#*:}"
case "$dest" in
  /*) ;;
  *) dest="$SHIM_FAKE_HOME/$dest" ;;
esac
mkdir -p "$dest"
# Local copy: this shim models the transfer, so the real rsync (which would
# spawn the ssh shim and recurse) is deliberately not used.
# `deployment/` is skipped so the fake Pi keeps its stub scripts and the test
# can observe the exact arguments deploy.sh passes them.
for entry in "$src"/*; do
  name="$(basename "$entry")"
  if [ "$name" = "deployment" ]; then
    continue
  fi
  cp -R "$entry" "$dest"/
done
"""


@pytest.fixture
def fake_pi(tmp_path):
    """A fake Pi layout with shims that execute remote commands locally."""
    fake_home = tmp_path / "home"
    fake_home.mkdir()
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    shim_log = tmp_path / "shim-log"
    shim_log.mkdir()

    for name, body in (("ssh", SSH_SHIM), ("rsync", RSYNC_SHIM)):
        path = bin_dir / name
        path.write_text(body, encoding="utf-8")
        path.chmod(0o755)

    # Local bundle: real deployment scripts plus a stub wheel directory.
    bundle = tmp_path / "local-bundle"
    shutil.copytree(REPO / "deployment", bundle / "deployment")
    shutil.copytree(
        REPO / "scripts", bundle / "scripts", dirs_exist_ok=True
    )
    (bundle / "wheels").mkdir()
    (bundle / "wheels" / "waferbot-0.1.0-py3-none-any.whl").write_bytes(b"fake")
    (bundle / "app").mkdir()
    (bundle / "app" / "pyproject.toml").write_text("[project]\nname='waferbot'\n")
    (bundle / "manifest.json").write_text(json.dumps({"files": []}))
    (bundle / "README.txt").write_text("fake bundle\n")

    remote_root = fake_home / "waferbot-bundle"
    remote_root.mkdir()
    # Stub the Pi-side scripts so the shims exercise paths/arguments, not apt.
    (remote_root / "deployment").mkdir()
    for name in ("install_pi.sh", "verify_hardware.sh"):
        stub = remote_root / "deployment" / name
        stub.write_text(
            "#!/usr/bin/env bash\n"
            'printf "%s %s\\n" "' + name + '" "$*" >> "$SHIM_LOG/calls.log"\n',
            encoding="utf-8",
        )
        stub.chmod(0o755)

    return {
        "home": fake_home,
        "bin": bin_dir,
        "log": shim_log,
        "bundle": bundle,
        "remote_root": remote_root,
        "tmp": tmp_path,
    }


def deploy_env(fake_pi) -> dict[str, str]:
    env = dict(os.environ)
    env["PATH"] = f"{fake_pi['bin']}:{env['PATH']}"
    env["HOME"] = str(fake_pi["home"])
    env["SHIM_LOG"] = str(fake_pi["log"])
    env["SHIM_FAKE_HOME"] = str(fake_pi["home"])
    return env


def test_deploy_remote_commands_use_correct_paths_and_arguments(fake_pi) -> None:
    result = subprocess.run(
        [
            "bash",
            str(REPO / "deployment" / "deploy.sh"),
            "--host", "fakehost",
            "--user", "tester",
            "--remote-dir", str(fake_pi["remote_root"]),
            "--bundle", str(fake_pi["bundle"]),
            "--venv", "/opt/venvs/waferbot",
        ],
        capture_output=True,
        text=True,
        env=deploy_env(fake_pi),
    )
    assert result.returncode == 0, result.stderr

    calls = (fake_pi["log"] / "calls.log").read_text(encoding="utf-8").splitlines()
    install_call = next(line for line in calls if line.startswith("install_pi.sh "))
    verify_call = next(line for line in calls if line.startswith("verify_hardware.sh "))

    remote = str(fake_pi["remote_root"])
    # The bundle path is the remote directory itself (never doubled).
    assert "--bundle ." in install_call
    assert f"cd '{remote}'" in (fake_pi["log"] / "ssh-commands.log").read_text()
    assert "--venv /opt/venvs/waferbot" in install_call
    assert "--yes" in install_call
    assert f"--venv /opt/venvs/waferbot" in verify_call

    # rsync really copied the bundle onto the "Pi".
    assert (fake_pi["remote_root"] / "wheels" / "waferbot-0.1.0-py3-none-any.whl").exists()
    assert (fake_pi["remote_root"] / "deployment").is_dir()
    assert (fake_pi["remote_root"] / "manifest.json").exists()

    # No command references the doubled path that the review found.
    assert f"{remote}/{remote}" not in install_call


def test_deploy_expands_home_on_the_pi_not_locally(fake_pi) -> None:
    result = subprocess.run(
        [
            "bash",
            str(REPO / "deployment" / "deploy.sh"),
            "--dry-run",
            "--host", "fakehost",
            "--remote-dir", str(fake_pi["remote_root"]),
            "--bundle", str(fake_pi["bundle"]),
        ],
        capture_output=True,
        text=True,
        env=deploy_env(fake_pi),
    )
    assert result.returncode == 0, result.stderr
    remote_lines = [
        line for line in result.stdout.splitlines() if line.startswith("REMOTE_CMD:")
    ]
    assert remote_lines
    assert all("$HOME/.venvs/waferbot" in line for line in remote_lines)
    # The local home from this machine must not have been substituted.
    assert str(Path.home()) not in result.stdout.replace(str(fake_pi["home"]), "")


def test_deploy_rejects_unsafe_venv_paths(fake_pi) -> None:
    result = subprocess.run(
        [
            "bash",
            str(REPO / "deployment" / "deploy.sh"),
            "--host", "fakehost",
            "--bundle", str(fake_pi["bundle"]),
            "--venv", "/tmp/x; rm -rf /",
        ],
        capture_output=True,
        text=True,
        env=deploy_env(fake_pi),
    )
    assert result.returncode == 2
    assert "unsafe" in result.stderr


def test_offline_install_never_upgrades_pip() -> None:
    text = (REPO / "deployment" / "offline_install.sh").read_text(encoding="utf-8")
    assert "--upgrade pip" not in text
    assert "--no-index" in text
    # Every pip invocation must be index-free (join shell line continuations).
    joined = text.replace("\\\n", " ")
    for chunk in joined.split("pip install")[1:]:
        assert "--no-index" in chunk


def test_install_pi_bundle_mode_never_upgrades_pip_online() -> None:
    text = (REPO / "deployment" / "install_pi.sh").read_text(encoding="utf-8")
    bundle_section = text.split("Installing from offline bundle")[1].split("elif")[0]
    assert "--upgrade pip" not in bundle_section
    assert 'bash "$SCRIPT_DIR/offline_install.sh"' in bundle_section
    offline = (REPO / "deployment/offline_install.sh").read_text()
    assert "--no-index" in offline
    assert "--find-links" in offline


def test_verify_hardware_propagates_bus_and_is_read_only() -> None:
    text = (REPO / "deployment" / "verify_hardware.sh").read_text(encoding="utf-8")
    assert "--config" in text
    assert "sensors --physical" in text
    assert "motor-test" not in text.split("Next steps")[0]


@pytest.mark.skipif(
    not (REPO / "dist" / "waferbot-offline-0.3.0").is_dir(),
    reason="offline bundle has not been built in this workspace",
)
def test_shipped_offline_installer_works_with_no_network(tmp_path) -> None:
    """Run the bundle's own script with PyPI pointed at an invalid address."""
    bundle = REPO / "dist" / "waferbot-offline-0.3.0"
    venv = tmp_path / "venv"
    env = dict(os.environ)
    env["PIP_INDEX_URL"] = "http://127.0.0.1:1/simple"
    env["PIP_RETRIES"] = "0"
    env["PIP_TIMEOUT"] = "1"
    result = subprocess.run(
        [
            "bash",
            str(bundle / "deployment" / "offline_install.sh"),
            "--bundle", str(bundle),
            "--venv", str(venv),
        ],
        capture_output=True,
        text=True,
        env=env,
        cwd=str(bundle),
    )
    assert result.returncode == 0, result.stderr
    installed = subprocess.run(
        [str(venv / "bin" / "python"), "-c", "import waferbot, smbus2; print('ok')"],
        capture_output=True,
        text=True,
    )
    assert installed.returncode == 0, installed.stderr
    assert "ok" in installed.stdout
    version = subprocess.check_output([str(venv / "bin/waferbot"), "--version"], text=True)
    assert version.strip() == "waferbot 0.3.0"


def test_default_relative_remote_directory_resolves_bundle(fake_pi):
    stub = fake_pi["remote_root"] / "deployment/install_pi.sh"
    stub.write_text(stub.read_text() + '\n[ "$1" = "--bundle" ] && [ -d "$2/wheels" ]\n')
    result = subprocess.run(
        ["bash", str(REPO / "deployment/deploy.sh"), "--host", "fakehost",
         "--bundle", str(fake_pi["bundle"])], capture_output=True, text=True,
        env=deploy_env(fake_pi),
    )
    assert result.returncode == 0, result.stderr
    calls = (fake_pi["log"] / "calls.log").read_text()
    assert "install_pi.sh --bundle ." in calls
