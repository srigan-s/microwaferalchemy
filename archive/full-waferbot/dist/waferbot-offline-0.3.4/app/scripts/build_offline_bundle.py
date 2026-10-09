#!/usr/bin/env python3
"""Build a reproducible offline bundle for the Raspberry Pi.

The bundle contains:

* ``app/``      - the allowlisted application files (source, maps, config,
                  deployment scripts, docs). Nothing from ``vendor/`` and no
                  credential-like file is ever copied.
* ``wheels/``   - the built ``waferbot`` wheel plus redistributable pure-Python
                  dependency wheels (``smbus2``).
* ``manifest.json`` - sha256 and size for every bundled file.
* ``bundle.json``   - what the bundle contains and how to install it.
* ``README.txt``    - the install commands for the Pi.

Reproducibility: the file list, the order of the manifest, and every recorded
hash are deterministic for a given source tree and dependency version. Wheel
bytes themselves may embed build timestamps.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Only these top-level entries are ever copied into the bundle.
ALLOWLIST: tuple[str, ...] = (
    "pyproject.toml",
    "README.md",
    "HARDWARE_IMPLEMENTATION.md",
    "docs/LINE_EDGE_TEST.md",
    "docs/APRILTAG_TESTING.md",
    "docs/ROBOT_TESTING.md",
    "docs/PHYSICAL_NAVIGATION_FILES.md",
    "LICENSE",
    "src",
    "maps",
    "config",
    "deployment",
    "scripts",
)
OPTIONAL_ALLOWLIST: tuple[str, ...] = ("tests",)

# Substrings that must never appear in a bundled path.
DENY_PATTERNS: tuple[str, ...] = (
    "vendor/",
    "api_key",
    "apikey",
    "secret",
    "credential",
    "password",
    "token",
    ".env",
    ".git/",
    "__pycache__",
    "appid",
)

# Redistributable, pure-Python runtime dependencies.
DEPENDENCY_REQUIREMENTS: tuple[str, ...] = ("smbus2",)


def read_version(repo: Path) -> str:
    pyproject = (repo / "pyproject.toml").read_text(encoding="utf-8")
    for line in pyproject.splitlines():
        stripped = line.strip()
        if stripped.startswith("version") and "=" in stripped:
            return stripped.split("=", 1)[1].strip().strip('"')
    return "0.0.0"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def copy_allowlisted(repo: Path, destination: Path, *, with_tests: bool) -> list[str]:
    names = list(ALLOWLIST) + (list(OPTIONAL_ALLOWLIST) if with_tests else [])
    copied: list[str] = []
    for name in names:
        source = repo / name
        if not source.exists():
            continue
        if source.is_dir():
            target = destination / name
            shutil.copytree(
                source,
                target,
                ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".DS_Store"),
                dirs_exist_ok=True,
            )
            for path in sorted(target.rglob("*")):
                if path.is_file():
                    copied.append(str(path.relative_to(destination)))
        else:
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            copied.append(str(target.relative_to(destination)))
    return copied


def assert_clean(bundle: Path) -> None:
    offenders: list[str] = []
    for path in bundle.rglob("*"):
        if not path.is_file():
            continue
        relative = str(path.relative_to(bundle)).lower()
        for pattern in DENY_PATTERNS:
            if pattern in relative:
                offenders.append(relative)
                break
    if offenders:
        raise SystemExit(
            "refusing to ship a bundle containing disallowed paths:\n  "
            + "\n  ".join(sorted(offenders))
        )


def run(command: list[str], *, cwd: Path | None = None) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)


def build_app_wheel(app_dir: Path, wheels_dir: Path, *, offline: bool = False) -> None:
    command = [
        sys.executable,
        "-m",
        "pip",
        "wheel",
        str(app_dir),
        "--no-deps",
        "--wheel-dir",
        str(wheels_dir),
    ]
    if offline:
        # Reuse the installed build backend instead of resolving it from PyPI.
        command.append("--no-build-isolation")
    run(command)


def download_dependencies(wheels_dir: Path, requirements: tuple[str, ...]) -> None:
    run(
        [
            sys.executable,
            "-m",
            "pip",
            "download",
            *requirements,
            "--only-binary=:all:",
            "--no-deps",
            "--dest",
            str(wheels_dir),
        ]
    )


def write_manifest(bundle: Path, *, version: str, notes: list[str]) -> None:
    files = []
    for path in sorted(bundle.rglob("*")):
        if not path.is_file():
            continue
        if path.name in {"manifest.json", "bundle.json"}:
            continue
        files.append(
            {
                "path": str(path.relative_to(bundle)),
                "size": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    manifest = {
        "bundle_version": version,
        "generated_by": "scripts/build_offline_bundle.py",
        "files": files,
        "file_count": len(files),
    }
    (bundle / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    wheels = []
    for wheel in sorted((bundle / "wheels").glob("*.whl")):
        wheels.append(
            {
                "filename": wheel.name,
                "sha256": sha256_file(wheel),
                "size": wheel.stat().st_size,
                "license": _wheel_license(wheel),
            }
        )
    bundle_info = {
        "name": bundle.name,
        "waferbot_version": version,
        "python_requires": ">=3.11",
        "platform": "any (pure-Python wheels only; no compiled extensions)",
        "installed_by": "deployment/offline_install.sh or deployment/install_pi.sh --bundle",
        "apt_prerequisites": [
            "python3-venv",
            "python3-dev",
            "i2c-tools",
            "libi2c-dev",
        ],
        "wheels": wheels,
        "notes": notes,
    }
    (bundle / "bundle.json").write_text(
        json.dumps(bundle_info, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def write_readme(bundle: Path, *, version: str) -> None:
    """Written before the manifest so its hash is recorded like everything else."""
    (bundle / "README.txt").write_text(
        "\n".join(
            [
                f"waferbot offline bundle {version}",
                "",
                "Layout:",
                "  app/        application source (allowlisted files only)",
                "  deployment/ install and verification scripts",
                "  wheels/     waferbot + pure-Python dependency wheels",
                "",
                "Install on a Raspberry Pi 5 running Raspberry Pi OS 64-bit:",
                "",
                "  bash deployment/offline_install.sh --bundle . --venv ~/.venvs/waferbot",
                "",
                "Prerequisites that must come from apt (not bundled here):",
                "  sudo apt-get install -y python3-venv python3-dev i2c-tools libi2c-dev",
                "",
                "Nothing is installed to start on boot. Verify before driving:",
                "  ~/.venvs/waferbot/bin/waferbot sensors --physical --samples 5",
                "",
                "I2C must be enabled (Control Centre > Interfaces > I2C, or",
                "sudo raspi-config > 3 Interface Options > I5 I2C).",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def _wheel_license(wheel: Path) -> str:
    """Best-effort licence metadata from the wheel's own dist-info."""
    try:
        with zipfile.ZipFile(wheel) as archive:
            for name in archive.namelist():
                if name.endswith(".dist-info/METADATA"):
                    for line in archive.read(name).decode(
                        "utf-8", errors="replace"
                    ).splitlines():
                        if line.lower().startswith("license"):
                            return line.split(":", 1)[1].strip() or "see wheel METADATA"
    except (zipfile.BadZipFile, OSError):
        return "unreadable; inspect the wheel metadata"
    return "see wheel METADATA"


def _verify_wheels(wheels_dir: Path) -> None:
    names = sorted(path.name for path in wheels_dir.glob("*.whl"))
    if not any(name.startswith("waferbot-") for name in names):
        raise SystemExit("bundle is missing the waferbot wheel")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="build an offline waferbot bundle")
    parser.add_argument("--output-dir", default=str(REPO_ROOT / "dist"))
    parser.add_argument("--version", default=None, help="override the package version")
    parser.add_argument(
        "--with-tests", action="store_true", help="also bundle the test suite"
    )
    parser.add_argument(
        "--no-download",
        action="store_true",
        help="build only the application wheel (reuse existing wheels offline)",
    )
    parser.add_argument(
        "--requirements",
        default=",".join(DEPENDENCY_REQUIREMENTS),
        help="comma-separated runtime requirements to download",
    )
    args = parser.parse_args(argv)

    version = args.version or read_version(REPO_ROOT)
    output_dir = Path(args.output_dir).resolve()
    bundle = output_dir / f"waferbot-offline-{version}"
    existing_wheels = sorted((bundle / "wheels").glob("*.whl")) if bundle.exists() else []
    if args.no_download and existing_wheels:
        # Truthful --no-download: reuse the wheels already in the bundle rather
        # than deleting them, and refresh only the application files.
        print(f"reusing {len(existing_wheels)} existing wheel(s) in {bundle}")
    else:
        if bundle.exists():
            shutil.rmtree(bundle)
        bundle.mkdir(parents=True)

    app_dir = bundle / "app"
    # Preserve cached dependency wheels, never stale application files.
    if app_dir.exists():
        shutil.rmtree(app_dir)
    app_dir.mkdir(exist_ok=True)
    wheels_dir = bundle / "wheels"
    wheels_dir.mkdir(exist_ok=True)
    for previous_app in wheels_dir.glob("waferbot-*.whl"):
        previous_app.unlink()

    copied = copy_allowlisted(REPO_ROOT, app_dir, with_tests=args.with_tests)
    print(f"copied {len(copied)} application files into {app_dir}")
    # Ship the deployment scripts at the bundle root as well, because every
    # documented command runs `bash deployment/<script>.sh` from the bundle.
    deployment_source = REPO_ROOT / "deployment"
    if deployment_source.is_dir():
        shutil.rmtree(bundle / "deployment", ignore_errors=True)
        shutil.copytree(
            deployment_source,
            bundle / "deployment",
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".DS_Store"),
        )
        print("copied deployment/ to the bundle root")

    build_app_wheel(app_dir, wheels_dir, offline=args.no_download)
    # `pip wheel` leaves a build/ tree and egg-info inside the source copy;
    # neither belongs in the shipped application files.
    shutil.rmtree(app_dir / "build", ignore_errors=True)
    for egg_info in app_dir.glob("**/*.egg-info"):
        shutil.rmtree(egg_info, ignore_errors=True)
    notes = [
        "Includes the application source and its built wheel.",
        "Third-party wheels are pure Python; no vendor directory is included.",
    ]
    if args.no_download:
        notes.append("Dependency download skipped (--no-download).")
    else:
        requirements = tuple(
            item.strip() for item in args.requirements.split(",") if item.strip()
        )
        download_dependencies(wheels_dir, requirements)
        notes.append("Downloaded dependencies: " + ", ".join(requirements))

    _verify_wheels(wheels_dir)
    write_readme(bundle, version=version)
    assert_clean(bundle)
    write_manifest(bundle, version=version, notes=notes)

    wheel_names = sorted(path.name for path in wheels_dir.glob("*.whl"))
    print(f"bundle: {bundle}")
    print(f"wheels: {', '.join(wheel_names) if wheel_names else '(none)'}")
    print("contents are manifest-verified; no credential or vendor paths included")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
