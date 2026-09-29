"""Shared fixtures. No test in this suite touches real hardware."""

from __future__ import annotations

import pytest

from waferbot import MockI2CTransport, Robot, RobotConfig


class FakeClock:
    """Deterministic monotonic clock for watchdog assertions."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class StepClock:
    """Monotonic clock that advances on every call (control-loop tests)."""

    def __init__(self, step: float = 0.05, start: float = 1000.0) -> None:
        self.step = step
        self.now = start

    def __call__(self) -> float:
        self.now += self.step
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def step_clock() -> StepClock:
    return StepClock()


@pytest.fixture
def no_sleep():
    """Replacement for time.sleep in control-loop tests."""

    def _sleep(_seconds: float) -> None:
        return None

    return _sleep


@pytest.fixture
def config() -> RobotConfig:
    return RobotConfig()


@pytest.fixture
def transport() -> MockI2CTransport:
    return MockI2CTransport()


@pytest.fixture
def robot(transport: MockI2CTransport, config: RobotConfig, clock: FakeClock) -> Robot:
    return Robot(transport, config, clock=clock)
