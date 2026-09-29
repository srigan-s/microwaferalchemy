"""The package must import without smbus2 and without touching hardware."""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys

import pytest

from waferbot import I2CError, Smbus2Transport
from waferbot.hardware.registers import I2C_ADDRESS_DEFAULT, I2C_BUS_DEFAULT


def test_import_does_not_pull_in_smbus2():
    code = (
        "import sys, waferbot;"
        "assert 'smbus2' not in sys.modules, 'smbus2 imported at import time';"
        "assert waferbot.__version__"
    )
    env = dict(os.environ)
    src = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
    env["PYTHONPATH"] = src + os.pathsep + env.get("PYTHONPATH", "")
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, env=env
    )
    assert result.returncode == 0, result.stderr


def test_protocol_defaults_match_vendor_source():
    assert I2C_ADDRESS_DEFAULT == 0x2B
    assert I2C_BUS_DEFAULT == 1


@pytest.mark.skipif(
    importlib.util.find_spec("smbus2") is not None,
    reason="smbus2 is installed; the missing-dependency path cannot be exercised",
)
def test_physical_transport_reports_missing_smbus2_clearly():
    with pytest.raises(I2CError) as excinfo:
        Smbus2Transport(1)
    assert "smbus2" in str(excinfo.value)
    assert isinstance(excinfo.value.__cause__, ImportError)

