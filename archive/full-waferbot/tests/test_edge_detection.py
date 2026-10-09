"""Edge classification, debouncing, freshness, and outer-channel handling."""

from __future__ import annotations

import pytest

from waferbot import SensorConfig, SensorError, decode_line_byte
from waferbot.errors import SensorError as ErrorsSensorError
from waferbot.sensing import (
    EdgeDetector,
    EdgeState,
    classify_middle,
    detect_edge,
    edge_byte_for,
)

LEFT = edge_byte_for(EdgeState.BLACK_LEFT)
RIGHT = edge_byte_for(EdgeState.BLACK_RIGHT)
BOTH_BLACK = edge_byte_for(EdgeState.BOTH_BLACK)
BOTH_WHITE = edge_byte_for(EdgeState.BOTH_WHITE)


def test_edge_bytes_match_the_documented_polarity():
    # S2 is bit 3 and S3 is bit 1; raw 0 means black.
    assert LEFT == 0b0111
    assert RIGHT == 0b1101
    assert BOTH_BLACK == 0b0101
    assert BOTH_WHITE == 0b1111


def test_classify_middle_covers_all_four_states():
    assert classify_middle((0, 1, 0, 0)) is EdgeState.BLACK_LEFT
    assert classify_middle((0, 0, 1, 0)) is EdgeState.BLACK_RIGHT
    assert classify_middle((0, 1, 1, 0)) is EdgeState.BOTH_BLACK
    assert classify_middle((0, 0, 0, 0)) is EdgeState.BOTH_WHITE
    with pytest.raises(SensorError):
        classify_middle((1, 1))


def test_detect_edge_accepts_byte_reading_and_channels():
    from_byte = detect_edge(LEFT)
    assert from_byte.edge is EdgeState.BLACK_LEFT

    reading = decode_line_byte(RIGHT)
    assert detect_edge(reading).edge is EdgeState.BLACK_RIGHT

    # Four raw channel bits, ordered S1..S4 (raw 0 = black).
    assert detect_edge((1, 0, 1, 1)).edge is EdgeState.BLACK_LEFT
    with pytest.raises(SensorError):
        detect_edge((2, 0, 1, 1))
    with pytest.raises(SensorError):
        detect_edge("nope")


def test_outer_channels_and_junction_flag():
    detection = detect_edge(edge_byte_for(EdgeState.BLACK_LEFT, outer=True))
    assert detection.outer_left is True
    assert detection.outer_right is False

    junction_bytes = 0x01  # S1, S2, S3 black, S4 white
    assert detect_edge(junction_bytes).junction is True
    assert detect_edge(LEFT).junction is False


def test_debounce_requires_consecutive_fresh_samples():
    detector = EdgeDetector(stable_samples=3, max_age_s=0.25)
    first = decode_line_byte(LEFT, timestamp=0.00)
    second = decode_line_byte(LEFT, timestamp=0.05)
    third_reading = decode_line_byte(LEFT, timestamp=0.10)
    assert detector.update(first, now=0.00).stable is False
    assert detector.update(second, now=0.05).stable is False
    third = detector.update(third_reading, now=0.10)
    assert third.stable is True
    assert third.stable_edge is EdgeState.BLACK_LEFT
    assert third.stable_count == 3


def test_replayed_reading_never_becomes_stable():
    """The review reproduction: one LineReading reused must not debounce."""
    detector = EdgeDetector(stable_samples=3, max_age_s=0.5)
    reading = decode_line_byte(LEFT, timestamp=1.0)
    results = [detector.update(reading, now=1.0) for _ in range(3)]
    assert [result.stable for result in results] == [False, False, False]
    assert all(result.rejected for result in results[1:])
    assert "duplicate" in (results[-1].reject_reason or "")


def test_backwards_and_gapped_evidence_resets_stability():
    detector = EdgeDetector(stable_samples=2, max_age_s=0.1)
    detector.update(decode_line_byte(LEFT, timestamp=1.00), now=1.00)
    backwards = detector.update(decode_line_byte(LEFT, timestamp=0.90), now=1.00)
    assert backwards.stable is False
    assert "duplicate or non-increasing" in (backwards.reject_reason or "")

    detector.reset()
    detector.update(decode_line_byte(LEFT, timestamp=2.00), now=2.00)
    gapped = detector.update(decode_line_byte(LEFT, timestamp=3.00), now=3.00)
    assert gapped.stable is False
    assert "gap" in (gapped.reject_reason or "")


def test_future_and_nonfinite_evidence_is_rejected():
    detector = EdgeDetector(stable_samples=2, max_age_s=0.1)
    future = detector.update(decode_line_byte(LEFT, timestamp=10.0), now=1.0)
    assert future.stable is False
    assert "future" in (future.reject_reason or "")
    with pytest.raises(SensorError):
        detector.update(decode_line_byte(LEFT, timestamp=float("nan")), now=1.0)


def test_invalid_channel_values_are_rejected():
    from waferbot.hardware.sensors import LineReading

    detector = EdgeDetector(stable_samples=1, max_age_s=1.0)
    bad = LineReading(
        raw_byte=0, raw=(0, 2, 0, 0), normalized=(0, 1, 0, 0), timestamp=1.0
    )
    with pytest.raises(SensorError):
        detector.update(bad, now=1.0)


def test_edge_byte_for_respects_configured_polarity():
    inverted = SensorConfig(black_is_raw_zero=False)
    # Raw 1 now means black, so the black channel bits are set.
    assert edge_byte_for(EdgeState.BLACK_LEFT, inverted) == 0b1000
    assert edge_byte_for(EdgeState.BLACK_RIGHT, inverted) == 0b0010
    assert edge_byte_for(EdgeState.BOTH_WHITE, inverted) == 0b0000
    assert classify_middle(
        decode_line_byte(edge_byte_for(EdgeState.BLACK_LEFT, inverted), inverted).normalized
    ) is EdgeState.BLACK_LEFT


def test_ambiguous_readings_reset_the_debounce_run():
    detector = EdgeDetector(stable_samples=2, max_age_s=0.25)
    detector.update(decode_line_byte(LEFT))
    flipped = detector.update(decode_line_byte(BOTH_BLACK))
    assert flipped.stable is False
    assert flipped.edge is EdgeState.BOTH_BLACK
    assert detector.update(decode_line_byte(LEFT)).stable is False


def test_stale_readings_never_become_stable():
    detector = EdgeDetector(stable_samples=2, max_age_s=0.1)
    old = decode_line_byte(LEFT, timestamp=0.0)
    detection = detector.update(old, now=5.0)
    assert detection.stale is True
    assert detection.stable is False
    assert detection.stable_edge is EdgeState.UNKNOWN
    assert detection.age_s == pytest.approx(5.0)


def test_detector_time_gap_breaks_the_run():
    detector = EdgeDetector(stable_samples=2, max_age_s=0.05)
    reading = decode_line_byte(LEFT)
    detector.update(reading, now=0.0)
    # Same pattern, but too long since the previous sample.
    late = detector.update(reading, now=1.0)
    assert late.stable is False


def test_configurable_polarity_reverses_black_and_white():
    inverted = SensorConfig(black_is_raw_zero=False)
    reading = decode_line_byte(0xFF, inverted)
    assert reading.normalized == (1, 1, 1, 1)
    assert classify_middle(reading.normalized) is EdgeState.BOTH_BLACK
    assert classify_middle(decode_line_byte(0xFF).normalized) is EdgeState.BOTH_WHITE


def test_edge_state_helpers():
    assert EdgeState.BLACK_LEFT.opposite is EdgeState.BLACK_RIGHT
    assert EdgeState.BLACK_LEFT.is_edge is True
    assert EdgeState.BOTH_BLACK.is_ambiguous is True
    with pytest.raises(ValueError):
        EdgeState.BOTH_WHITE.opposite
    assert EdgeState.from_name("black-left") is EdgeState.BLACK_LEFT
    assert EdgeState.from_name("BLACK_RIGHT") is EdgeState.BLACK_RIGHT
    with pytest.raises(ValueError):
        EdgeState.from_name("middle")


def test_detect_edge_detector_argument_is_used():
    detector = EdgeDetector(stable_samples=2, max_age_s=1.0)
    detect_edge(LEFT, detector=detector)
    second = detect_edge(LEFT, detector=detector)
    assert second.stable is True
    assert detector.last is second


def test_sensor_error_import_is_consistent():
    assert ErrorsSensorError is SensorError


def test_small_future_offset_rejected_without_poisoning_later_evidence():
    detector = EdgeDetector(stable_samples=2, max_age_s=0.25)
    assert detector.update(decode_line_byte(LEFT, timestamp=1.1), now=1.0).rejected
    assert not detector.update(decode_line_byte(LEFT, timestamp=1.01), now=1.01).rejected
    assert detector.update(decode_line_byte(LEFT, timestamp=1.02), now=1.02).stable


def test_backward_frame_cannot_make_a_replay_look_new():
    detector = EdgeDetector(stable_samples=2, max_age_s=0.25)
    detector.update(decode_line_byte(LEFT, timestamp=1.1), now=1.1)
    assert detector.update(decode_line_byte(LEFT, timestamp=1.0), now=1.1).rejected
    assert detector.update(decode_line_byte(LEFT, timestamp=1.1), now=1.1).rejected
    assert not detector.update(decode_line_byte(LEFT, timestamp=1.12), now=1.12).stable
    assert detector.update(decode_line_byte(LEFT, timestamp=1.13), now=1.13).stable
