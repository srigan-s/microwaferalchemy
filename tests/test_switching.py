"""Edge-switching FSM: authorization, phases, verification, and failures."""

from __future__ import annotations

import time

import pytest

from waferbot import (
    ConfigError,
    FaultCode,
    MockI2CTransport,
    Robot,
    RobotConfig,
    SafetyError,
)
from waferbot.errors import AuthorizationError
from waferbot.localization import Localization
from waferbot.navconfig import GeometryConfig, SwitchConfig
from waferbot.sensing import EdgeState, SwitchAuthorization, SwitchPhase, edge_byte_for
from waferbot.sensing.switching import EdgeSwitcher

LEFT = edge_byte_for(EdgeState.BLACK_LEFT)
RIGHT = edge_byte_for(EdgeState.BLACK_RIGHT)
STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def make_robot(sequence, *, clock, watchdog: bool = False):
    transport = MockI2CTransport(line_sensor_sequence=list(sequence))
    robot = Robot(transport, RobotConfig(), clock=clock, watchdog=watchdog)
    return robot, transport


def fixed_provider(node: str, *, clock=time.monotonic):
    def provider(expected: str):
        return Localization(
            node_id=expected,
            marker_id=expected,
            confidence=1.0,
            timestamp=clock(),
            source="fixed",
        )

    return provider


class FixedMonitor:
    def __init__(self, node: str, *, clock=time.monotonic) -> None:
        self.node = node
        self.queries: list[str] = []
        self._clock = clock

    def requires_stop(self) -> bool:
        return False

    def check(self, expected_node: str):
        self.queries.append(expected_node)
        if expected_node != self.node:
            return None
        return Localization(
            node_id=expected_node,
            marker_id=expected_node,
            confidence=1.0,
            timestamp=self._clock(),
            source="fixed",
        )


class NullMonitor:
    def __init__(self) -> None:
        self.queries = 0

    def requires_stop(self) -> bool:
        return False

    def check(self, expected_node: str):
        self.queries += 1
        return None


def make_authorization(**overrides) -> SwitchAuthorization:
    payload = {
        "authorized": True,
        "route_id": "route-1",
        "source_node": "B2",
        "source_edge": EdgeState.BLACK_LEFT,
        "target_edge": EdgeState.BLACK_RIGHT,
        "location_id": "B2",
        "destination_location_id": "B3",
        "issued_at": None,
    }
    payload.update(overrides)
    return SwitchAuthorization(**payload)


def make_switcher(
    robot,
    *,
    step_clock,
    no_sleep,
    config: SwitchConfig | None = None,
    provider=None,
    geometry: GeometryConfig | None = None,
    confirm=None,
):
    return EdgeSwitcher(
        robot,
        config=config or SwitchConfig(),
        geometry=geometry or GeometryConfig(),
        clock=step_clock,
        sleep=no_sleep,
        location_provider=provider,
        confirm=confirm,
    )


# -- happy path --------------------------------------------------------------


def test_successful_lateral_switch_runs_every_required_phase(
    step_clock, no_sleep
) -> None:
    # Model fast clock reads, below the 250ms sensor sampling-gap limit.
    step_clock.step = 0.005
    robot, transport = make_robot([LEFT] * 10 + [RIGHT] * 40, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    monitor = FixedMonitor("B3", clock=step_clock)

    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=make_authorization(issued_at=step_clock()),
        location_id="B2",
        destination_monitor=monitor,
    )

    phases = [phase for phase, _stamp in result.phases]
    assert result.completed is True, result.reason
    assert phases == [
        SwitchPhase.FOLLOW_EDGE.value,
        SwitchPhase.APPROACH_SWITCH_LOCATION.value,
        SwitchPhase.CONFIRM_SWITCH_LOCATION.value,
        SwitchPhase.REDUCE_SPEED.value,
        SwitchPhase.EXECUTE_SWITCH.value,
        SwitchPhase.VERIFY_OPPOSITE_EDGE.value,
        SwitchPhase.RESUME_FOLLOWING.value,
        SwitchPhase.COMPLETE.value,
    ]
    assert result.verification_samples >= 3
    assert result.destination is not None
    assert result.destination.node_id == "B3"
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS
    assert robot.fault is None


def test_switch_starts_with_a_bounded_lateral_strafe(step_clock, no_sleep) -> None:
    robot, transport = make_robot([LEFT] * 10 + [RIGHT] * 40, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=make_authorization(issued_at=step_clock()),
        location_id="B2",
        destination_monitor=FixedMonitor("B3", clock=step_clock),
    )

    groups = [transport.motor_payloads[i : i + 4] for i in range(0, len(transport.motor_payloads), 4)]

    def signature(group):
        return tuple(1 if payload[1] == 0 else -1 for payload in group)

    # BLACK_LEFT means the tape body is on the left, so crossing strafes left.
    assert any(signature(group) == (-1, 1, 1, -1) for group in groups)


# -- authorization failures --------------------------------------------------


def test_unauthorized_switch_is_refused(step_clock, no_sleep) -> None:
    robot, transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    with pytest.raises(AuthorizationError):
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=make_authorization(authorized=False),
            location_id="B2",
        )
    assert transport.motor_payloads == []


@pytest.mark.parametrize(
    "overrides",
    [
        {"source_edge": EdgeState.BLACK_RIGHT},  # wrong current edge
        {"target_edge": EdgeState.BLACK_LEFT},  # wrong target
        {"location_id": "C2"},  # wrong location binding
        {"source_node": "C2"},  # source node / location mismatch
    ],
)
def test_mismatched_authorization_is_refused(
    step_clock, no_sleep, overrides
) -> None:
    robot, transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    with pytest.raises(AuthorizationError):
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=make_authorization(**overrides),
            location_id="B2",
        )
    assert transport.motor_payloads == []


def test_stale_authorization_is_refused(step_clock, no_sleep) -> None:
    robot, _transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    stale = step_clock() - 600.0
    with pytest.raises(AuthorizationError) as excinfo:
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=make_authorization(issued_at=stale),
            location_id="B2",
        )
    assert "stale" in str(excinfo.value)


def test_missing_location_is_refused(step_clock, no_sleep) -> None:
    robot, _transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot, step_clock=step_clock, no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    with pytest.raises(AuthorizationError):
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=make_authorization(),
            location_id=None,
        )


def test_destination_confirmation_required_but_missing_source(
    step_clock, no_sleep
) -> None:
    robot, _transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    switcher = make_switcher(robot, step_clock=step_clock, no_sleep=no_sleep)
    with pytest.raises(AuthorizationError):
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=make_authorization(),
            location_id="B2",
        )


def test_switch_requires_armed_robot(step_clock, no_sleep) -> None:
    robot, _transport = make_robot([LEFT] * 40, clock=step_clock)
    switcher = make_switcher(
        robot, step_clock=step_clock, no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    with pytest.raises(SafetyError):
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=make_authorization(),
            location_id="B2",
        )


# -- localization and manoeuvre failures -------------------------------------


def test_current_edge_must_be_established_first(step_clock, no_sleep) -> None:
    # Sensors only ever show the right edge, so the left edge cannot be proven.
    robot, transport = make_robot([RIGHT] * 200, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot, step_clock=step_clock, no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=make_authorization(issued_at=step_clock()),
        location_id="B2",
        destination_monitor=FixedMonitor("B3", clock=step_clock),
    )
    assert result.completed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LOCALIZATION_FAILURE
    assert "establish" in result.reason
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS


def test_unconfirmed_switch_location_fails(step_clock, no_sleep) -> None:
    robot, _transport = make_robot([LEFT] * 100, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=lambda expected: None,
    )
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=make_authorization(issued_at=step_clock()),
        location_id="B2",
        destination_monitor=FixedMonitor("B3", clock=step_clock),
    )
    assert result.completed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LOCALIZATION_FAILURE


def test_opposite_edge_never_found_is_a_switch_timeout(step_clock, no_sleep) -> None:
    robot, transport = make_robot([LEFT] * 300, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot, step_clock=step_clock, no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=make_authorization(issued_at=step_clock()),
        location_id="B2",
        destination_monitor=FixedMonitor("B3", clock=step_clock),
    )
    assert result.completed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.SWITCH_TIMEOUT
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS


def test_transient_opposite_edge_does_not_verify(step_clock, no_sleep) -> None:
    # Two right-edge readings in a row are not enough to confirm the switch.
    robot, _transport = make_robot([LEFT] * 12 + [RIGHT] * 2 + [LEFT] * 200, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot, step_clock=step_clock, no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=make_authorization(issued_at=step_clock()),
        location_id="B2",
        destination_monitor=FixedMonitor("B3", clock=step_clock),
    )
    assert result.completed is False
    assert result.verification_samples < 3
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.SWITCH_TIMEOUT


def test_destination_must_be_confirmed_after_crossing(step_clock, no_sleep) -> None:
    robot, _transport = make_robot([LEFT] * 10 + [RIGHT] * 200, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot, step_clock=step_clock, no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
    )
    monitor = NullMonitor()
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=make_authorization(issued_at=step_clock()),
        location_id="B2",
        destination_monitor=monitor,
    )
    assert result.completed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LOCALIZATION_FAILURE
    assert monitor.queries >= 1


# -- diagonal mode -----------------------------------------------------------


MEASURED = GeometryConfig(
    tape_width_m=0.02,
    sensor_spacing_m=0.012,
    sensor_detection_width_m=0.004,
    available_crossing_distance_m=0.20,
    safety_margin_m=0.01,
    robot_width_m=0.15,
    switching_speed_mps=0.1,
    sampling_rate_hz=20.0,
    lateral_clearance_m=0.3,
    required_confirmations=3,
    measured=True,
)


def test_diagonal_mode_needs_measured_geometry(step_clock, no_sleep) -> None:
    robot, transport = make_robot([LEFT] * 10 + [RIGHT] * 200, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
        config=SwitchConfig(mode="diagonal", crossing_angle_deg=20.0),
        geometry=GeometryConfig(),
    )
    with pytest.raises(ConfigError):
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=make_authorization(issued_at=step_clock()),
            location_id="B2",
            destination_monitor=FixedMonitor("B3", clock=step_clock),
        )
    assert transport.motor_payloads == []


def test_diagonal_mode_with_unsafe_angle_is_refused(step_clock, no_sleep) -> None:
    robot, _transport = make_robot([LEFT] * 10 + [RIGHT] * 200, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
        config=SwitchConfig(mode="diagonal", crossing_angle_deg=3.0),
        geometry=MEASURED,
    )
    with pytest.raises(ConfigError):
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=make_authorization(issued_at=step_clock()),
            location_id="B2",
            destination_monitor=FixedMonitor("B3", clock=step_clock),
        )


def test_diagonal_mode_moves_forward_and_sideways(step_clock, no_sleep) -> None:
    # Model fast clock reads, below the 250ms sensor sampling-gap limit.
    step_clock.step = 0.005
    robot, transport = make_robot([LEFT] * 10 + [RIGHT] * 200, clock=step_clock)
    robot.arm()
    switcher = make_switcher(
        robot,
        step_clock=step_clock,
        no_sleep=no_sleep,
        provider=fixed_provider("B2", clock=step_clock),
        config=SwitchConfig(mode="diagonal", crossing_angle_deg=20.0),
        geometry=MEASURED,
    )
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=make_authorization(issued_at=step_clock()),
        location_id="B2",
        destination_monitor=FixedMonitor("B3", clock=step_clock),
    )
    assert result.completed is True
    assert result.geometry is not None
    assert result.geometry["unambiguous_geometry"] is True

    groups = [transport.motor_payloads[i : i + 4] for i in range(0, len(transport.motor_payloads), 4)]
    # Diagonal crossing: every wheel keeps a forward component.
    diagonal_groups = [
        group
        for group in groups
        if all(payload[1] == 0 and payload[2] > 0 for payload in group)
    ]
    assert diagonal_groups
