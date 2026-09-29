"""CSV telemetry fields, units, and honest handling of uncalibrated speeds."""

from __future__ import annotations

from waferbot.telemetry import (
    CSV_FIELDS,
    NullTelemetry,
    TelemetryLogger,
    open_telemetry,
    read_rows,
)
from waferbot.localization import Localization
from waferbot.safety import FaultCode, FaultRecord
from waferbot.hardware.sensors import decode_line_byte


def test_header_contains_the_requested_fields(tmp_path):
    path = tmp_path / "telemetry.csv"
    with TelemetryLogger(path) as logger:
        logger.log("test")
    header = path.read_text(encoding="utf-8").splitlines()[0]
    for field in (
        "raw",
        "normalized",
        "detected_edge",
        "target_edge",
        "current_node",
        "target_node",
        "motor_m1",
        "estimated_speed_pwm",
        "estimated_speed_mps",
        "fault",
    ):
        assert field in header.split(",")
    assert list(CSV_FIELDS) == header.split(",")


def test_rows_carry_sensor_motor_and_fault_data(tmp_path):
    path = tmp_path / "telemetry.csv"
    with TelemetryLogger(path) as logger:
        logger.sensor_reading(decode_line_byte(0x07))
        logger.motor_command((40, 40, 30, 30), context="edge_follow")
        logger.control(
            event="follow",
            detected_edge="BLACK_LEFT",
            target_edge="BLACK_LEFT",
            current_node="A+",
            target_node="B+",
            stable=True,
        )
        logger.localization(
            Localization(
                node_id="B+",
                marker_id="B+",
                confidence=1.0,
                timestamp=1.0,
                source="marker",
            ),
            expected="B+",
        )
        logger.fault(FaultRecord(FaultCode.LINE_LOST, "lost the edge", 2.0))

    rows = read_rows(path)
    assert [row["event"] for row in rows] == [
        "sensor",
        "motor",
        "follow",
        "localization",
        "fault",
    ]
    sensor_row = rows[0]
    assert sensor_row["raw"] == "1 0 1 1"
    assert sensor_row["normalized"] == "0 1 0 0"
    motor_row = rows[1]
    assert motor_row["motor_m1"] == "40"
    assert motor_row["motor_m4"] == "30"
    assert motor_row["estimated_speed_pwm"] == "35.00"
    assert motor_row["estimated_speed_mps"] == ""
    assert rows[2]["detected_edge"] == "BLACK_LEFT"
    assert rows[2]["stable"] == "1"
    assert rows[3]["current_node"] == "B+"
    assert rows[4]["fault"] == "LINE_LOST"


def test_speed_conversion_requires_a_measured_factor(tmp_path):
    path = tmp_path / "telemetry.csv"
    logger = TelemetryLogger(path, counts_to_mps=0.01)
    logger.motor_command((50, 50, 50, 50), context="follow")
    logger.close()
    row = read_rows(path)[0]
    assert row["estimated_speed_pwm"] == "50.00"
    assert row["estimated_speed_mps"] == "0.5000"


def test_unknown_fields_are_preserved_in_note(tmp_path):
    path = tmp_path / "telemetry.csv"
    logger = TelemetryLogger(path)
    logger.log("custom", phase="EXECUTE_SWITCH", extra="value")
    logger.close()
    row = read_rows(path)[0]
    assert row["phase"] == "EXECUTE_SWITCH"
    assert "extra=value" in row["note"]


def test_logger_appends_and_counts(tmp_path):
    path = tmp_path / "telemetry.csv"
    first = TelemetryLogger(path)
    first.log("one")
    first.close()
    second = TelemetryLogger(path)
    second.log("two")
    second.close()
    rows = read_rows(path)
    assert [row["event"] for row in rows] == ["one", "two"]


def test_null_telemetry_is_a_no_op():
    telemetry = open_telemetry(None)
    assert isinstance(telemetry, NullTelemetry)
    telemetry.log("ignored")
    telemetry.motor_command((1, 2, 3, 4))
    telemetry.close()
    assert telemetry.rows_written == 0


def test_stop_and_switch_phase_events_are_logged(tmp_path):
    path = tmp_path / "telemetry.csv"
    with TelemetryLogger(path) as logger:
        logger.stop("hold-end")
        logger.switch_phase("EXECUTE_SWITCH", target_edge="BLACK_RIGHT")
        logger.control(
            event="switch_result",
            detected_edge=None,
            target_edge="BLACK_RIGHT",
            phase="FAULT",
            note="switch timeout",
        )
    rows = read_rows(path)
    assert rows[0]["event"] == "stop"
    assert rows[0]["motor_context"] == "hold-end"
    assert rows[1]["event"] == "switch_phase"
    assert rows[1]["phase"] == "EXECUTE_SWITCH"
    # A failed crossing must not claim the target edge was observed.
    assert rows[2]["detected_edge"] == ""
    assert rows[2]["target_edge"] == "BLACK_RIGHT"


def test_close_is_idempotent_and_late_logs_are_dropped(tmp_path):
    path = tmp_path / "telemetry.csv"
    logger = TelemetryLogger(path)
    logger.log("one")
    logger.close()
    logger.close()  # must not raise
    logger.log("after-close")  # must not raise (dropped)
    rows = read_rows(path)
    assert [row["event"] for row in rows] == ["one"]


def test_concurrent_logging_is_serialised(tmp_path):
    import threading

    path = tmp_path / "telemetry.csv"
    logger = TelemetryLogger(path)
    threads = [
        threading.Thread(target=lambda index=i: [logger.log(f"t{index}") for _ in range(20)])
        for i in range(4)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    logger.close()
    rows = read_rows(path)
    assert len(rows) == 80
