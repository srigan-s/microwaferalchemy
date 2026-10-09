"""Deployment scripts and bundle generator: syntax, allowlist, and safety."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
SCRIPTS = [
    REPO / "deployment" / "install_pi.sh",
    REPO / "deployment" / "offline_install.sh",
    REPO / "deployment" / "verify_hardware.sh",
    REPO / "deployment" / "deploy.sh",
]


@pytest.mark.parametrize("script", SCRIPTS, ids=lambda path: path.name)
def test_shell_scripts_pass_bash_syntax_check(script):
    assert script.exists(), f"missing deployment script {script}"
    if shutil.which("bash") is None:  # pragma: no cover - bash is present on CI
        pytest.skip("bash not available")
    result = subprocess.run(
        ["bash", "-n", str(script)], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("script", SCRIPTS, ids=lambda path: path.name)
def test_shell_scripts_are_executable(script):
    assert script.stat().st_mode & 0o111, f"{script.name} is not executable"


def test_installer_never_installs_an_autostart_service():
    text = (REPO / "deployment" / "install_pi.sh").read_text(encoding="utf-8")
    assert "systemctl enable" not in text
    assert "cron" not in text
    assert "rc.local" not in text
    assert "Nothing was installed to start on boot" in (
        REPO / "deployment" / "install_pi.sh"
    ).read_text(encoding="utf-8")


def test_deploy_script_has_no_erase_or_format_commands():
    text = (REPO / "deployment" / "deploy.sh").read_text(encoding="utf-8")
    for forbidden in ("dd if=", "mkfs", "diskutil", "fdisk", "parted"):
        assert forbidden not in text
    assert "--delete" not in text


def test_bundle_generator_allowlist_excludes_vendor_and_credentials():
    text = (REPO / "scripts" / "build_offline_bundle.py").read_text(encoding="utf-8")
    assert '"vendor"' not in text.replace("'vendor'", '"vendor"')
    assert "vendor/" in text  # deny pattern
    assert "api_key" in text  # deny pattern
    assert "ALLOWLIST" in text
    assert "manifest.json" in text


def test_bundle_generator_help_runs():
    result = subprocess.run(
        [sys.executable, str(REPO / "scripts" / "build_offline_bundle.py"), "--help"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "--output-dir" in result.stdout


def test_deployment_readme_exists_and_mentions_no_erase_tooling():
    text = (REPO / "deployment" / "README.md").read_text(encoding="utf-8")
    assert "never erase" in text
    assert "verify_hardware.sh" in text

