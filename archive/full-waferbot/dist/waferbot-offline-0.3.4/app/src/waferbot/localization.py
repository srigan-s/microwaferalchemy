"""Localization interfaces, a mock localizer, and manual confirmation.

Arrival is never inferred from elapsed time or from edge polarity. The route
executor asks a localizer for a node identity, and only a positive match (or an
explicit operator confirmation with the wheels stopped) counts as arrival.

Future QR-code or fiducial-marker readers only have to implement
:class:`ArrivalMonitor`; nothing else in the navigation stack changes.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .errors import LocalizationError


@dataclass(frozen=True)
class Localization:
    """A positive identification of where the robot is."""

    node_id: str | None
    marker_id: str
    confidence: float
    timestamp: float
    source: str = "marker"

    def age_s(self, now: float) -> float:
        return now - self.timestamp

    def is_fresh(self, now: float, max_age_s: float) -> bool:
        age = self.age_s(now)
        return 0 <= age <= max_age_s

    def matches(
        self,
        expected_node: str,
        *,
        marker_to_node: dict[str, str] | None = None,
    ) -> bool:
        """True only when this evidence genuinely identifies ``expected_node``.

        A positive ``node_id`` is authoritative: if it names a different node, a
        coincidentally matching ``marker_id`` must not be accepted. A
        marker-only localization either matches the expected id directly or maps
        through the caller's marker-to-node table.
        """
        if self.node_id is not None:
            return self.node_id == expected_node
        if self.marker_id == expected_node:
            return True
        if marker_to_node is not None:
            return marker_to_node.get(self.marker_id) == expected_node
        return False


@dataclass(frozen=True)
class LocalizationPolicy:
    """Validation rules applied to every localization observation."""

    max_age_s: float = 5.0
    min_confidence: float = 0.5

    def reject_reason(
        self,
        localization: "Localization | None",
        *,
        expected: str | None,
        now: float,
        marker_to_node: dict[str, str] | None = None,
    ) -> str | None:
        """Return a human-readable rejection reason, or ``None`` when valid."""
        if localization is None:
            return "no localization evidence"
        if not isinstance(localization.confidence, (int, float)) or isinstance(
            localization.confidence, bool
        ):
            return "confidence is not a number"
        confidence = float(localization.confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            return f"confidence {confidence!r} is not a finite value in 0..1"
        if confidence < self.min_confidence:
            return (
                f"confidence {confidence:.2f} is below the {self.min_confidence:.2f} "
                "minimum"
            )
        timestamp = localization.timestamp
        if (
            isinstance(timestamp, bool)
            or not isinstance(timestamp, (int, float))
            or not math.isfinite(timestamp)
        ):
            return "timestamp is not finite"
        age = now - float(timestamp)
        if age < 0:
            return f"timestamp is {abs(age):.2f}s in the future"
        if age > self.max_age_s:
            return f"evidence is stale ({age:.2f}s old)"
        if expected is not None and not localization.matches(
            expected, marker_to_node=marker_to_node
        ):
            return (
                f"evidence identifies "
                f"{localization.node_id or localization.marker_id!r}, "
                f"not {expected!r}"
            )
        return None


class LocalizationConfirmer:
    """Collects independent, strictly newer evidence for one expected node.

    Duplicate frames (same timestamp), backwards timestamps, stale or
    low-confidence frames, and frames naming the wrong node are all rejected and
    never count towards the required sample count.
    """

    def __init__(
        self,
        expected: str,
        *,
        policy: LocalizationPolicy,
        required: int = 1,
        clock: Callable[[], float] = time.monotonic,
        marker_to_node: dict[str, str] | None = None,
    ) -> None:
        if required < 1:
            raise ValueError("required must be >= 1")
        self.expected = expected
        self.policy = policy
        self.required = int(required)
        self._clock = clock
        self._marker_to_node = marker_to_node
        self.accepted: list[Localization] = []
        self.rejections: list[str] = []
        self._last_timestamp: float | None = None

    @property
    def complete(self) -> bool:
        return len(self.accepted) >= self.required

    @property
    def latest(self) -> Localization | None:
        return self.accepted[-1] if self.accepted else None

    def reset(self) -> None:
        self.accepted.clear()
        self.rejections.clear()
        self._last_timestamp = None

    def offer(self, localization: "Localization | None") -> bool:
        """Validate one observation. Returns True when it was accepted."""
        now = self._clock()
        reason = self.policy.reject_reason(
            localization,
            expected=self.expected,
            now=now,
            marker_to_node=self._marker_to_node,
        )
        if reason is None and localization is not None:
            timestamp = float(localization.timestamp)
            if self._last_timestamp is not None and timestamp <= self._last_timestamp:
                reason = (
                    "evidence timestamp is not newer than the previous accepted "
                    "frame (duplicate or replayed frame)"
                )
        if reason is not None:
            self.rejections.append(reason)
            return False
        assert localization is not None  # for type checkers
        self.accepted.append(localization)
        self._last_timestamp = float(localization.timestamp)
        return True

    def offer_until_complete(
        self, provider: Callable[[str], "Localization | None"], deadline: float, sleep
    ) -> bool:
        """Poll ``provider`` for this node until the policy is satisfied."""
        while self._clock() < deadline:
            if self.offer(provider(self.expected)) and self.complete:
                return True
            sleep(0.0)
        return self.complete


@runtime_checkable
class ArrivalMonitor(Protocol):
    """Confirms whether the robot is at ``expected_node``."""

    def requires_stop(self) -> bool:
        """True when the check must happen with the wheels stopped."""

    def check(self, expected_node: str) -> Localization | None:
        """Return a localization when the robot is at ``expected_node``."""


class MarkerLocalizer:
    """Base class for marker readers (QR codes, fiducials, reflective tags)."""

    def requires_stop(self) -> bool:
        return False

    def read_marker(self) -> Localization | None:  # pragma: no cover - interface
        raise NotImplementedError

    def check(self, expected_node: str) -> Localization | None:
        found = self.read_marker()
        if found is not None and found.matches(expected_node):
            return found
        return None


class MockArrivalMonitor:
    """Deterministic monitor for tests and mock sessions.

    Each ``check`` for a given node advances an internal counter; the monitor
    reports arrival only after ``polls_before_arrival`` checks, so a mock route
    exercises the same control loop a marker reader would.
    """

    def __init__(
        self,
        *,
        polls_before_arrival: int = 2,
        clock: Callable[[], float] = time.monotonic,
        requires_stop: bool = False,
    ) -> None:
        if polls_before_arrival < 1:
            raise ValueError("polls_before_arrival must be >= 1")
        self.polls_before_arrival = int(polls_before_arrival)
        self._counts: dict[str, int] = {}
        self._clock = clock
        self._requires_stop = requires_stop
        self.queries: list[str] = []

    def requires_stop(self) -> bool:
        return self._requires_stop

    def check(self, expected_node: str) -> Localization | None:
        self.queries.append(expected_node)
        count = self._counts.get(expected_node, 0) + 1
        self._counts[expected_node] = count
        if count < self.polls_before_arrival:
            return None
        return Localization(
            node_id=expected_node,
            marker_id=expected_node,
            confidence=1.0,
            timestamp=self._clock(),
            source="mock",
        )

    def reset(self) -> None:
        self._counts.clear()
        self.queries.clear()


class ScriptedLocalizer:
    """Replays a fixed sequence of node ids, one per ``advance_every`` polls."""

    def __init__(
        self,
        sequence: Iterable[str],
        *,
        advance_every: int = 1,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.sequence: list[str] = list(sequence)
        self.advance_every = max(1, int(advance_every))
        self._index = 0
        self._polls = 0
        self._clock = clock

    @property
    def current(self) -> str | None:
        if not self.sequence:
            return None
        return self.sequence[min(self._index, len(self.sequence) - 1)]

    def requires_stop(self) -> bool:
        return False

    def check(self, expected_node: str) -> Localization | None:
        node = self.current
        self._polls += 1
        if self._polls >= self.advance_every:
            self._polls = 0
            self._index = min(self._index + 1, max(0, len(self.sequence) - 1))
        if node is None or node != expected_node:
            return None
        return Localization(
            node_id=node,
            marker_id=node,
            confidence=1.0,
            timestamp=self._clock(),
            source="scripted",
        )


class ManualArrivalMonitor:
    """Operator confirmation, always with the wheels stopped."""

    def __init__(
        self,
        prompt: Callable[[str], bool],
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._prompt = prompt
        self._clock = clock
        self.queries: list[str] = []

    def requires_stop(self) -> bool:
        return True

    def check(self, expected_node: str) -> Localization | None:
        return self.check_with_context(expected_node, "arrival")

    def check_with_context(self, expected_node: str, purpose: str) -> Localization | None:
        self.queries.append(expected_node)
        if not self._prompt(
            f"{purpose}: Is the robot at node {expected_node}? Motors are stopped; answer"
            " only after looking at the track."
        ):
            return None
        return Localization(
            node_id=expected_node,
            marker_id=expected_node,
            confidence=1.0,
            timestamp=self._clock(),
            source="manual",
        )


class SequenceLocalizer:
    """A localizer built from explicit measurements instead of a live marker.

    Useful for tests: pass the exact sequence of node ids the robot should
    report, and every node is reported once.
    """

    def __init__(self, sequence: Sequence[str]) -> None:
        self.sequence = list(sequence)
        self._index = 0

    def requires_stop(self) -> bool:
        return False

    def check(self, expected_node: str) -> Localization | None:
        if self._index >= len(self.sequence):
            return None
        node = self.sequence[self._index]
        self._index += 1
        if node != expected_node:
            return None
        return Localization(
            node_id=node,
            marker_id=node,
            confidence=1.0,
            timestamp=time.monotonic(),
            source="sequence",
        )


def require_localization(
    monitor: ArrivalMonitor | None, expected_node: str, *, purpose: str
) -> Localization:
    """Ask ``monitor`` once and raise when it cannot confirm."""
    if monitor is None:
        raise LocalizationError(
            f"{purpose}: no localization source is configured for {expected_node}"
        )
    found = monitor.check(expected_node)
    if found is None:
        raise LocalizationError(
            f"{purpose}: could not confirm position {expected_node}"
        )
    return found


__all__ = [
    "ArrivalMonitor",
    "Localization",
    "LocalizationConfirmer",
    "LocalizationPolicy",
    "ManualArrivalMonitor",
    "MarkerLocalizer",
    "MockArrivalMonitor",
    "ScriptedLocalizer",
    "SequenceLocalizer",
    "require_localization",
]
