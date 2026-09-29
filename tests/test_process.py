"""Process-safe ownership, stop latch, and cross-process bus locking."""

from __future__ import annotations

import json
import os

import pytest

from waferbot.errors import SafetyError
from waferbot.process import ProcessGuard


def test_ownership_is_exclusive(tmp_path):
    first = ProcessGuard(tmp_path)
    info = first.acquire_ownership("motor-test")
    assert info.pid == os.getpid()
    assert info.command == "motor-test"
    assert first.holds_ownership is True
    assert first.session_is_live() is True

    second = ProcessGuard(tmp_path)
    with pytest.raises(SafetyError) as excinfo:
        second.acquire_ownership("follow")
    assert "already running" in str(excinfo.value)

    first.release_ownership()
    assert first.holds_ownership is False
    third = ProcessGuard(tmp_path)
    third.acquire_ownership("follow")
    third.release_ownership()


def test_session_record_is_readable_and_cleaned_up(tmp_path):
    guard = ProcessGuard(tmp_path)
    guard.acquire_ownership("execute")
    assert guard.session() is not None
    assert json.loads((tmp_path / "owner.json").read_text())["command"] == "execute"
    guard.release_ownership()
    assert guard.session() is None
    assert not (tmp_path / "owner.json").exists()
    assert not list(tmp_path.glob("*.tmp*"))


def test_stop_latch_round_trip(tmp_path):
    guard = ProcessGuard(tmp_path)
    assert guard.stop_requested() is False
    assert guard.stop_request() is None

    request = guard.request_stop("operator stop")
    assert request.reason == "operator stop"
    assert guard.stop_requested() is True
    assert ProcessGuard(tmp_path).stop_requested() is True

    reloaded = ProcessGuard(tmp_path).stop_request()
    assert reloaded is not None
    assert reloaded.pid == os.getpid()

    assert guard.clear_stop_request() is True
    assert guard.stop_requested() is False
    assert guard.clear_stop_request() is False
    assert not list(tmp_path.glob("*.tmp*"))


def test_bus_guard_is_exclusive_with_timeout(tmp_path):
    holder = ProcessGuard(tmp_path / "a")
    waiter = ProcessGuard(tmp_path / "a")
    with holder.bus_guard(timeout_s=1.0):
        with pytest.raises(TimeoutError):
            with waiter.bus_guard(timeout_s=0.05):
                pass
    # Once released, the lock is available again.
    with waiter.bus_guard(timeout_s=0.5):
        pass


def test_session_scoped_runtime_directories_are_isolated(tmp_path):
    one = ProcessGuard(tmp_path, session_name="one")
    two = ProcessGuard(tmp_path, session_name="two")
    one.acquire_ownership("one")
    two.acquire_ownership("two")
    assert one.runtime_dir != two.runtime_dir
    one.release_ownership()
    two.release_ownership()

