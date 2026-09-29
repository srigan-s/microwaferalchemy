"""Localization evidence validation: freshness, identity, and no replay."""

from __future__ import annotations

import math

import pytest

from waferbot.localization import (
    Localization,
    LocalizationConfirmer,
    LocalizationPolicy,
    ManualArrivalMonitor,
    MockArrivalMonitor,
    ScriptedLocalizer,
)

POLICY = LocalizationPolicy(max_age_s=1.0, min_confidence=0.5)


def make(node: str | None = "B+", *, marker: str | None = None, confidence=1.0, ts=10.0, source="test"):
    return Localization(
        node_id=node,
        marker_id=marker if marker is not None else (node or "unknown"),
        confidence=confidence,
        timestamp=ts,
        source=source,
    )


def test_matches_rejects_a_marker_that_disagrees_with_a_positive_node_id():
    evidence = Localization(
        node_id="C-", marker_id="B+", confidence=1.0, timestamp=0.0
    )
    # A coincidentally matching marker must not override a positive node id.
    assert evidence.matches("B+") is False
    assert evidence.matches("C-") is True


def test_matches_maps_marker_only_evidence_through_the_table():
    evidence = Localization(
        node_id=None, marker_id="QR-17", confidence=1.0, timestamp=0.0
    )
    assert evidence.matches("B+") is False
    assert evidence.matches("B+", marker_to_node={"QR-17": "B+"}) is True


@pytest.mark.parametrize(
    ("evidence", "needle"),
    [
        (None, "no localization"),
        (make(confidence=float("nan")), "confidence"),
        (make(confidence=float("inf")), "confidence"),
        (make(confidence=-0.1), "confidence"),
        (make(confidence=1.5), "confidence"),
        (make(confidence=0.2), "below"),
        (make(ts=float("nan")), "timestamp"),
        (make(ts=100.0), "future"),
        (make(ts=0.0), "stale"),
        (make(node="C-"), "not 'B+'"),
    ],
)
def test_policy_rejections(evidence, needle):
    reason = POLICY.reject_reason(evidence, expected="B+", now=10.0)
    assert reason is not None
    assert needle in reason


def test_policy_accepts_valid_evidence():
    assert POLICY.reject_reason(make(ts=9.8), expected="B+", now=10.0) is None
    assert POLICY.reject_reason(make(), expected=None, now=10.0) is None


def test_confirmer_requires_strictly_newer_evidence():
    confirmer = LocalizationConfirmer(
        "B+", policy=POLICY, required=3, clock=lambda: 10.0
    )
    first = make(ts=9.0)
    assert confirmer.offer(first) is True
    # Replaying the same frame must not count again.
    assert confirmer.offer(first) is False
    assert "newer" in confirmer.rejections[-1]
    assert confirmer.offer(make(ts=8.0)) is False
    assert confirmer.offer(make(ts=9.5)) is True
    assert confirmer.offer(make(ts=9.8)) is True
    assert confirmer.complete is True
    assert confirmer.latest.timestamp == 9.8
    assert len(confirmer.accepted) == 3


def test_confirmer_rejects_wrong_node_and_low_confidence():
    confirmer = LocalizationConfirmer(
        "B+", policy=POLICY, required=1, clock=lambda: 10.0
    )
    assert confirmer.offer(make(node="C-", ts=9.9)) is False
    assert confirmer.offer(make(confidence=0.1, ts=9.9)) is False
    assert confirmer.complete is False
    assert confirmer.offer(make(ts=9.9)) is True
    assert confirmer.complete is True


def test_mock_monitor_frames_are_valid_and_advance():
    monitor = MockArrivalMonitor(polls_before_arrival=2, clock=lambda: 5.0)
    assert monitor.check("B+") is None
    found = monitor.check("B+")
    assert found is not None and found.node_id == "B+"
    confirmer = LocalizationConfirmer(
        "B+", policy=POLICY, required=2, clock=lambda: 5.0
    )
    # Two identical frames from a frozen clock are the same evidence, not two.
    assert confirmer.offer(found) is True
    assert confirmer.offer(found) is False
    assert confirmer.complete is False


def test_manual_monitor_requires_stop_and_reports_manual_source():
    monitor = ManualArrivalMonitor(lambda _p: True, clock=lambda: 3.0)
    assert monitor.requires_stop() is True
    found = monitor.check("A+")
    assert found is not None
    assert found.source == "manual"
    assert found.timestamp == 3.0


def test_scripted_localizer_advances_and_reports_expected_only():
    localizer = ScriptedLocalizer(["A+", "B+"], advance_every=1, clock=lambda: 1.0)
    assert localizer.check("A+") is not None
    assert localizer.check("A+") is None
    assert localizer.check("B+") is not None


def test_even_small_future_localization_is_invalid():
    evidence = make(ts=10.001)
    assert not evidence.is_fresh(10.0, 1.0)
    assert "future" in POLICY.reject_reason(evidence, expected="B+", now=10.0)
