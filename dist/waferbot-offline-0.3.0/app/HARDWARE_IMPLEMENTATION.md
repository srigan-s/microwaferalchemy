# HARDWARE_IMPLEMENTATION.md

Physical hardware layer for the MicroAlchemy wafer transport system, targeting a
Yahboom Raspbot V2 (Raspberry Pi 5, four-wheel mecanum chassis, four-channel IR
line sensor).

## 1. Status and scope

This document describes the 0.3.0 physical navigation implementation.
The source is software-tested; physical calibration and track measurements
remain operator tasks. See `docs/ROBOT_TESTING.md` for the first-run procedure.

Implemented:

* I2C protocol constants and payload builders for the Raspbot V2 controller.
* A strict, injectable transport with a real `smbus2` backend and an in-memory
  mock.
* A motor driver with bounded speeds, no swallowed I2C errors, and best-effort
  all-wheel stops on partial command failure, plus per-wheel polarity inversion
  from calibration.
* A four-channel line sensor reader with normalisation and configurable
  polarity / channel mapping.
* An ultrasonic distance reader and an obstacle-monitor thread that feeds
  `FaultCode.OBSTACLE_DETECTED` into the safety controller.
* Safety primitives: explicit arming, typed latched faults, latched emergency
  stop, a motion deadline plus an **independent watchdog thread**, and
  process-safe ownership/stop coordination for the CLI.
* Sensing and control: debounced four-channel edge detection, a 20 Hz
  configurable PD `EdgeFollower`, and an `EdgeSwitcher` finite state machine
  binding authorization, source location, fresh readings, and destination
  confirmation.
* Geometry evaluation for crossing angles, a JSON track graph with strict
  validation, Dijkstra/A* planning with ordered waypoints, a localization-driven
  `RouteExecutor`, CSV telemetry, and calibration utilities.
* A `waferbot` CLI (mock by default, explicit physical confirmation), deployment
  scripts, and a reproducible offline bundle generator.
* Automated tests: hardware-free cases covering protocol, faults, watchdog,
  following, switching, geometry, graph, planning, execution, CLI, telemetry,
  process coordination, calibration, and configuration.

**No claim of physical hardware testing is made anywhere in this document.** See
section 12 for the confirmation steps that still need a real Pi.

## 2. Evidence inventory

Everything below was read from vendored source already present in this
repository. No external ZIP was opened, extracted, or inspected.

| Evidence | Location |
| --- | --- |
| Controller I2C address and bus | `vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py:7,19` |
| Register write helper | `vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py:34-40` |
| Register read helper | `vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py:49-55` |
| Signed motor command | `vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py:74-88` (`Ctrl_Muto`) |
| Explicit direction motor command | `vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py:58-72` (`Ctrl_Car`) |
| Line sensor parse (commented reference) | `vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py:399-405` |
| Wheel mixing formula | `vendor/yahboom/project_demo/lib/McLumk_Wheel_Sports.py:149-163` (`set_deflection`) |
| Forward / backward / strafe / rotation primitives | `vendor/yahboom/project_demo/lib/McLumk_Wheel_Sports.py:16-92` |
| Stop implementation | `vendor/yahboom/project_demo/lib/McLumk_Wheel_Sports.py:134-147` |
| Motor id -> wheel label | `vendor/yahboom/project_demo/03.Basic_car_course/5.Motor control.ipynb:46-73` |
| Line sensor register read + printed channel order | `vendor/yahboom/project_demo/03.Basic_car_course/7.Status of four-way line patrol module.ipynb:55-70` |
| Channel naming, outer/middle roles, black polarity | `vendor/yahboom/project_demo/05.Comprehensive_gameplay/1.infrared_patrol_line.ipynb:33-79` |
| Same follower used with obstacle stop | `vendor/yahboom/project_demo/05.Comprehensive_gameplay/4.linetrack_ultraavoid.ipynb` |
| Motion demos per primitive | `vendor/yahboom/project_demo/04.Car_motion_control/*.ipynb` |
| Semantic names (strafe vs rotate in place) | `vendor/yahboom/project_demo/09.AI_Big_Model/AI_CarAgent_en/Car_base_control.py:49-78` |

The vendored driver is written for `smbus` (`python3-smbus`). This package uses
`smbus2`, whose `SMBus` exposes the same `write_i2c_block_data` /
`read_i2c_block_data` calls, so the wire behaviour is identical.

## 3. Bus and transport

* Controller address: `0x2B` (`PI5Car_I2CADDR`).
* Bus: I2C bus `1` (`smbus.SMBus(1)`).
* Writes use block writes: `write_i2c_block_data(addr, reg, [..])`.
* Reads use block reads: `read_i2c_block_data(addr, reg, length)`.

`waferbot.hardware.transport.Smbus2Transport` implements exactly these two calls.
`smbus2` is imported lazily inside the constructor, so importing `waferbot`,
constructing a `Robot` over a mock, and running the test suite never require
`smbus2` and never touch `/dev/i2c-*`.

Any transfer failure is converted to `waferbot.errors.I2CError` with the original
exception chained as `__cause__`. The vendored driver instead prints
`'I2C error'` and continues, which is why an independent adapter exists.

## 4. Motor protocol

One block write per wheel, on register `0x01`, payload `[motor_id, direction, speed]`:

| Field | Values | Notes |
| --- | --- | --- |
| `motor_id` | `0..3` | see section 5 |
| `direction` | `0` = forward, `1` = backward | from `Ctrl_Muto` / `Ctrl_Car` |
| `speed` | `0..255` | magnitude; vendored code clamps, this adapter refuses |

`Ctrl_Muto(motor_id, motor_speed)` derives the direction from the sign and sends
`abs(motor_speed)`. `Ctrl_Car(motor_id, motor_dir, motor_speed)` takes the
direction explicitly, which is what the vendored `stop_robot()` uses
(`Ctrl_Car(i, 0, 0)` for `i` in `0..3`).

Consequences encoded in `waferbot.hardware.registers`:

* `motor_block(id, +60) == [id, 0, 60]`, `motor_block(id, -60) == [id, 1, 60]`.
* `stop_block(id) == [id, 0, 0]` (the vendored stop bytes).
* Out-of-range motor ids and speeds raise `ValueError`; they are never silently
  clamped, because a clamp hides caller bugs.

The adapter does not implement the vendored clamping of `motor_dir` values other
than 0/1; it only ever emits 0 or 1 by construction.

## 5. Motor id to wheel mapping

Motor ids are fixed by the vendored source and notebooks:

| Motor id | Vendored label | Physical position (assumed) |
| --- | --- | --- |
| `0` | `L1` | front-left |
| `1` | `L2` | rear-left |
| `2` | `R1` | front-right |
| `3` | `R2` | rear-right |

Evidence for the labels: `Raspbot_Lib.py` comments name the ids (`#L1电机`, etc.)
and `03.Basic_car_course/5.Motor control.ipynb:48-51` calls `Ctrl_Muto(0..3)`
with the four sliders in that order, while lines 70-73 stop the same ids labelled
`L1/L2/R1/R2`.

Evidence for the left/right grouping is strong: every vendored motion assigns
`l1, l2` to ids 0 and 1 and `r1, r2` to ids 2 and 3.

**Unverified:** which of `L1`/`L2` is the front wheel. No vendored file states
it, and the debug prints in `McLumk_Wheel_Sports.py` place the movement key on
the same row regardless of direction, so they do not establish front/rear. The
assumption above (`1` = front, `2` = rear) is cosmetic for phase 1: none of the
exposed primitives distinguish front from rear. It matters for diagonal moves
and must be confirmed before phase 2 adds them. The mapping lives in
`MotorConfig.wheel_labels` so it can be corrected without touching code.

## 6. Movement mixing

`set_deflection(speed, deflection)` in `McLumk_Wheel_Sports.py:149-163` defines
the chassis geometry as a top-down plane with `0 = right`, `90 = forward`,
`180 = left`, `270 = backward`:

```
vx = speed * cos(deflection)
vy = speed * sin(deflection)
l1 = int(vy + vx)
l2 = int(vy - vx)
r1 = int(vy - vx)
r2 = int(vy + vx)
```

At the four cardinal deflections the trig terms are exact for integer speeds, so
the vendor formula reduces to the following wheel patterns (motor ids 0, 1, 2, 3;
`+` = forward, `-` = backward):

| Action | Vendor call | Wheel pattern | Strafe/rotate character |
| --- | --- | --- | --- |
| `forward(speed)` | `set_deflection(speed, 90)` | `(+, +, +, +)` | — |
| `backward(speed)` | `set_deflection(speed, 270)` | `(-, -, -, -)` | — |
| `strafe_left(speed)` | `set_deflection(speed, 180)` | `(-, +, +, -)` | lateral translation |
| `strafe_right(speed)` | `set_deflection(speed, 0)` | `(+, -, -, +)` | lateral translation |
| `rotate_left(speed)` | `set_deflection(speed, 180)`, `l2 -> -l2`, `r2 -> abs(r2)` | `(-, -, +, +)` | differential spin |
| `rotate_right(speed)` | `set_deflection(speed, 0)`, `l2 -> abs(l2)`, `r2 -> -r2` | `(+, +, -, -)` | differential spin |

The rotation rows are exactly what `rotate_left` / `rotate_right` in
`McLumk_Wheel_Sports.py:74-92` emit: they reuse the strafing deflection but then
force both left wheels to share one sign and both right wheels to the opposite
sign. `09.AI_Big_Model/AI_CarAgent_en/Car_base_control.py:49-78` names the two
families separately ("Turn left in place" vs "Left translation"), which confirms
that strafing and rotating are intended to be different wheel patterns.

These six patterns are the whole of `waferbot.kinematics.WHEEL_SIGNATURES`; the
tests assert the exact bytes for each one.

## 7. Line sensor protocol

Read: `read_i2c_block_data(0x2B, 0x0A, 1)` returns one byte holding the four
channel bits. `Raspbot_Lib.py:399-405` and
`03.Basic_car_course/7.Status of four-way line patrol module.ipynb:55-70` both
decode it as:

```
x1 = (track >> 3) & 0x01
x2 = (track >> 2) & 0x01
x3 = (track >> 1) & 0x01
x4 = track & 0x01
```

### 7.1 Polarity

Raw `0` means the black line is detected; raw `1` means white / no line.

* `05.Comprehensive_gameplay/1.infrared_patrol_line.ipynb:47` treats an all-zero
  byte as "all black".
* Line 51 of the same notebook annotates a state with
  `0表示检测到黑线 ... 0 means black line is detected`.

Normalisation required by the project is therefore `BLACK = 1`, `WHITE = 0`, i.e.
`normalized = 1 - raw_bit`. This is implemented as `SensorConfig.black_is_raw_zero`
so an inverted board can be handled by configuration instead of a code change.

### 7.2 Channel order

The follower assigns physical roles to the four bits
(`1.infrared_patrol_line.ipynb:33-79`):

```
lineL1 = x2   # left outermost   ("左最外侧检测")
lineL2 = x1
lineR1 = x3
lineR2 = x4   # right outermost  ("右最外侧检测")
```

`7.Status of four-way line patrol module.ipynb:65-70` prints the channels in the
order `x2 x1 x3 x4` under the header `x2 x1 x3 x4`, matching the same left-to-right
layout. Combined with the bit extraction above, the physical order is:

| Physical channel | Bit | Role in phase 2 |
| --- | --- | --- |
| S1 (leftmost) | bit 2 | outer left, larger deviations |
| S2 | bit 3 | primary middle detector (BLACK_LEFT) |
| S3 | bit 1 | primary middle detector (BLACK_RIGHT) |
| S4 (rightmost) | bit 0 | outer right, larger deviations |

This is `waferbot.config.DEFAULT_SENSOR_BITS == (2, 3, 1, 0)` and is configurable
via `SensorConfig.bit_for_channel` (validated to be a permutation of `0..3`).

**Confidence and residual uncertainty:** the polarity claim is well supported by
in-line comments in two independent notebooks. The exact `bit -> physical
position` mapping is inferred from one follower notebook plus one display string,
which is weaker evidence. Confirm it with the phase 2 `waferbot sensors`
diagnostic by placing black tape under exactly one channel at a time, then adjust
`bit_for_channel` if needed. Because the mapping is configuration, no algorithm
change is required.

## 8. Other controller registers

Recorded from vendored source so later phases do not re-derive them:

| Register | Direction | Purpose |
| --- | --- | --- |
| `0x01` | write | motor command (implemented) |
| `0x02` | write | servo, payload `[id, angle]` |
| `0x03` / `0x04` | write | RGB LED all / single |
| `0x05` | write | IR remote switch |
| `0x06` | write | buzzer switch |
| `0x07` | write | ultrasonic ranging switch |
| `0x08` / `0x09` | write | LED brightness all / single |
| `0x0A` | read | line sensor byte (implemented) |
| `0x0C` | read | IR remote value |
| `0x1A` / `0x1B` | read | ultrasonic distance low / high byte |

The motor path (`0x01`), the line sensor path (`0x0A`), and the ultrasonic
enable/read path (`0x07`, `0x1A`/`0x1B`) are implemented. The LED, servo, buzzer,
and IR-remote registers are constants only: driving them without need would add
untested motor-adjacent behaviour.

## 9. Software interfaces

```
src/waferbot/
  protocol.py              leaf constants (address, registers, limits)
  errors.py                WaferbotError, I2CError, MotorCommandError,
                           SensorError, SafetyError, SpeedLimitError, ConfigError
  config.py                MotorConfig, SensorConfig, SafetyConfig, RobotConfig
  navconfig.py             FollowConfig, SwitchConfig, GeometryConfig,
                           ExecutionConfig, NavConfig
  kinematics.py            DriveAction, WHEEL_SIGNATURES, wheel_speeds()
  safety.py                SafetyState, FaultCode, FaultRecord, SafetyController
  watchdog.py              MotionWatchdog (independent deadline thread)
  process.py               ProcessGuard (ownership, stop latch, bus lock)
  localization.py          Localization, ArrivalMonitor, mock/scripted/manual
  geometry.py              CrossingInputs, minimum_crossing_angle_deg,
                           evaluate_crossing
  telemetry.py             TelemetryLogger (CSV), NullTelemetry
  robot.py                 Robot facade (movement + sensing + safety)
  cli.py                   `waferbot` command line
  hardware/
    transport.py           I2CTransport protocol, Smbus2Transport
    registers.py           motor_block, stop_block, motor_blocks
    motor_driver.py        MotorDriver
    sensors.py             LineReading, LineSensorArray, decode_line_byte
    ultrasonic.py          UltrasonicSensor, ObstacleMonitor
    bus.py                 BusLock, null_bus_guard
    mock.py                MockI2CTransport, MockLineSensorScript, build_mock_robot
  sensing/
    edge.py                EdgeState, EdgeDetector, detect_edge, edge_byte_for
    follower.py            EdgeFollower, lateral_error, wheel_command
    switching.py           EdgeSwitcher, SwitchAuthorization, SwitchPhase
  nav/
    graph.py               TrackMap, MapNode, MapEdge, NodeType, EdgeAction
    planner.py             Planner, Route, plan_route (Dijkstra + A*)
    executor.py            RouteExecutor, ExecutionResult
  calibration/
    wheels.py              WheelCalibrator (per-wheel order/polarity probes)
    sensors.py             calibrate_channels (bit order + polarity)
    crossing.py            CrossingSweep (angle success rate + CSV)
    speed.py               compute_counts_to_mps (measured, conservative)
```

The navigation-facing movement interface requested by the project:

```python
robot.forward(speed)      robot.backward(speed)
robot.strafe_left(speed)  robot.strafe_right(speed)
robot.rotate_left(speed)  robot.rotate_right(speed)
robot.stop()
```

All six motion methods return the per-wheel signed speeds they issued, which
keeps telemetry and tests straightforward. Additional public surface:

```python
robot.arm()                      # explicit permission to move
robot.disarm()
robot.read_line_sensors()        # LineReading, allowed while disarmed
robot.tick()                     # enforce the motion deadline; True if it fired
robot.raise_fault(code, message) # latches a FaultCode and stops the wheels
robot.clear_fault()
robot.emergency_stop(code, message)
robot.clear_emergency_stop(confirm=True)
robot.state                      # SafetyState
```

`speed` is a PWM magnitude in counts, never a velocity. Counts become metres per
second only when a measured `nav.counts_to_mps` factor exists; until then
telemetry leaves `estimated_speed_mps` empty.

## 10. Safety contract

Enforced by `Robot` and `SafetyController`:

1. **No motion on import or construction.** Building a transport, driver, sensor
   reader, or `Robot` emits zero I2C writes. Tested.
2. **Explicit arming.** Every motion method raises `SafetyError` unless
   `arm()` was called and no fault or emergency stop is latched.
3. **Bounded speed.** `MotorConfig.max_speed` (default 100) is the navigation
   bound and 255 is the protocol bound; violations raise `SpeedLimitError`
   before any bus traffic.
4. **Typed, latched faults.** `FaultCode` covers `LINE_LOST`, `SENSOR_FAILURE`,
   `SWITCH_TIMEOUT`, `LOCALIZATION_FAILURE`, `MOTOR_COMMUNICATION_FAILURE`, and
   `OBSTACLE_DETECTED`. A fault disarms the robot until `clear_fault()`.
5. **Latched emergency stop.** `emergency_stop()` latches first and then commands
   the stop, so a failing stop still leaves the robot refusing motion.
   `clear_emergency_stop(confirm=True)` is explicit and never re-arms by itself.
6. **Motion deadline watchdog.** Each motion command refreshes a deadline
   (`SafetyConfig.motion_timeout_s`, default 0.5 s). `robot.tick()` stops all
   wheels once it passes, and an **independent watchdog thread**
   (`waferbot.watchdog.MotionWatchdog`) does the same even when the controller
   never calls `tick()` again. The watchdog latches `FaultCode.MOTION_TIMEOUT`
   and takes the shared bus lock, so its stop cannot interleave with a
   four-wheel command.
7. **Best-effort stop on partial failure.** If a wheel write fails mid-command,
   the driver still attempts a stop for all four wheels, then raises
   `MotorCommandError` with the original `I2CError` chained. The robot also
   latches `MOTOR_COMMUNICATION_FAILURE`, stops again, and stays disarmed.
8. **No swallowed I2C errors.** Every transport failure propagates as
   `I2CError`; the vendored `print('I2C error')` behaviour is not reproduced.
9. **Failed stops persist.** `disarm()`, `stop()`, and a failed stop inside fault
   handling all attempt a stop; if that stop fails the robot records
   `MOTOR_COMMUNICATION_FAILURE` (or extends the existing fault message) and
   stays disarmed. Sensor read failures latch `SENSOR_FAILURE` and stop.
10. **Strict counts.** Wheel speeds must be integers: fractional or boolean
    values raise instead of being silently coerced into a command.
11. **One bus, one lock.** `motor.bus` and `sensor.bus` must match, and every
    bus access is serialised on a shared re-entrant lock.
12. **Process-safe stop.** A motion session owns the bus (exclusive `flock` and
    an `owner.json` record); `waferbot stop` latches a stop request that every
    motor command checks, takes the cross-process bus lock, and keeps motion
    refused until `waferbot stop --clear`.
13. **Graph speeds are enforced in m/s.** Physical route execution refuses to
    start unless a measured `nav.counts_to_mps` factor exists; each edge's
    `speed_limit_mps` is then converted to a PWM cap for that step, so a map
    limit is a real limit rather than a planner-only number. Mock sessions and
    `--dry-run` do not need the factor.
14. **Floor operation needs verified assumptions.** `motor.verified` and
    `sensor.verified` must be true (set after `waferbot calibrate wheels` and
    `waferbot calibrate sensors`) before `follow`, `switch`, `execute`, or a
    crossing sweep will move on the floor; `--ack-verified-config` is the explicit
    operator override. Wheel diagnostics stay permissioned by the
    wheels-lifted prompt and are reported as uncertain until confirmed.
15. **One transaction per command.** Permission checks, the external-stop latch,
    the arming state, the deadline refresh, and the wheel write all happen inside
    a single transaction that holds the in-process lock *and* the cross-process
    bus guard. A stop or watchdog that latches mid-command can never be followed
    by a nonzero write, and nested stop paths reuse the held guard instead of
    deadlocking.
16. **Tick never clears a live deadline.** `Robot.tick()` only acts when the
    deadline has actually expired, and then it latches `MOTION_TIMEOUT` and stops
    exactly like the independent watchdog thread.
17. **`waferbot stop --physical` writes stop bytes.** It takes the cross-process
    bus lock and sends the four stop blocks itself, so an absent or dead owner
    still leaves the controller stopped. It reports bytes written, lock state,
    and errors instead of claiming the lock proves the motors stopped, and it
    refuses `--clear` while a live session still owns the bus.
18. **Localization evidence is validated, never fabricated.** Physical execution
    accepts only operator confirmation or a real marker reader; every accepted
    observation must identify the expected node, carry a finite confidence above
    the configured minimum, be fresh, and be strictly newer than the previous
    accepted frame (replays do not count). The executor records the evidence the
    localizer produced, not a synthesised `source="arrival"`.
19. **Bounded, refreshed manoeuvres.** Wheel probes, TURN/DOCK, follower
    recovery, and switch phases re-issue their command faster than the watchdog
    timeout and respect the caller's remaining budget; a stop failure during
    cleanup makes the operation report failure.
20. **Graph speeds stay enforceable.** Physical execution requires a measured
    `counts_to_mps`; per-edge `speed_limit_mps` becomes a PWM cap for FOLLOW,
    SWITCH, TURN, and DOCK. A cap below one count is refused instead of being
    rounded up, and diagonal blends are scaled so no wheel exceeds the cap.
21. **Geometry is conservative and measured.** The evaluation keeps the
    `atan` bound and adds measured lateral clearance, a lateral-speed /
    sampling-rate / detection-footprint / confirmation-count observability
    check, and forward clearance. Physical geometry is evaluated against the
    commanded counts and the real polling rate, not against unrelated numbers.

**Limits of software safety:** nothing here can cut motor power. A killed
process, a wedged I2C bus, or a power fault leaves the controller board holding
its last command, because no independent hardware watchdog is verified on this
board. Physical motor power cutoff (switch, relay, or battery disconnect) remains
an operator requirement, and the robot must be tested with its wheels off the
ground first.

## 11. Configuration

`RobotConfig` is a frozen dataclass tree with strict validation (unknown keys,
non-finite timeouts, out-of-range speeds, non-permutation bit maps, and a
`motor.bus`/`sensor.bus` mismatch are all rejected) and JSON round-trip:

```json
{
  "motor": { "address": 43, "bus": 1, "max_speed": 100,
             "wheel_labels": ["front_left", "rear_left", "front_right", "rear_right"],
             "invert": [false, false, false, false] },
  "sensor": { "address": 43, "bus": 1, "register": 10,
              "bit_for_channel": [2, 3, 1, 0], "black_is_raw_zero": true },
  "safety": { "motion_timeout_s": 0.5, "watchdog_period_s": 0.05 }
}
```

`wheel_labels` is documentation and future-proofing, not a hardware address: only
the id order (0, 1, 2, 3) reaches the bus. `invert` is the polarity correction
that `waferbot calibrate wheels` reports.

Navigation configuration (`NavConfig`) is a separate tree in
`waferbot.navconfig` with strict validation: follow gains/rate/bounds, switch
mode and timeouts, measured geometry, execution bounds, telemetry path, map path,
and the optional measured speed factor.

## 12. Hardware confirmation checklist (on the Pi)

None of these has been executed here; each needs the real chassis.

1. `i2cdetect -y 1` shows the controller at `0x2B`; log the actual address.
2. Raise the wheels off the ground, then command one wheel at a time through
   `Ctrl_Muto`-equivalent bytes and record which physical corner moves and in
   which direction. This fixes `wheel_labels` and confirms the forward/backward
   direction byte.
3. Run `waferbot motor-test --physical --speed 5` with the wheels lifted and
   answer the prompts. The report stays `uncertain_until_confirmed` until every
   wheel is confirmed; set `motor.invert` for any wheel that ran backwards.
4. Place black tape under exactly one sensor channel at a time and print the raw
   byte (`waferbot sensors --physical`, `waferbot calibrate sensors`). This
   confirms `black_is_raw_zero` and `bit_for_channel`.
5. Measure tape width, sensor spacing, detection width, available crossing
   distance, robot width, crossing speed, and sampling rate; fill
   `nav.geometry` and set `measured: true` before any diagonal switch.
6. Run a straight-line follow (`waferbot follow --physical --edge black-left`),
   then a lateral switch with an explicit authorization, then
   `waferbot calibrate crossing --physical --angles ...`.
7. Time a straight run over a measured distance and record it with
   `waferbot calibrate speed` if `speed_limit_mps` should mean anything.
8. Replace the example map with a measured one (`is_example: false`,
   `physical_validated: true`, `edge_side_map` filled), then execute
   `waferbot execute --physical --start A+ --goal D- --via B- --localizer manual`.
9. Confirm the physical motor power cutoff works while the software is driving,
   and confirm `waferbot stop` halts a session from a second terminal.

## 13. Verification in this repository

Automated tests (mock hardware only) cover exact protocol bytes for all six
primitives and the stop path, motor-id ordering, speed bounding, partial-command
fault stops with error chaining, sensor polarity, channel mapping, malformed
frames, read failures, arming rules, fault latching, emergency-stop latching and
clearing, the independent watchdog, strict wheel counts, process-safe stop
coordination, edge debouncing and stale data, the follower (gains, junction
slowdown, bounded recovery, line-lost latch, final stop), the switch FSM
(authorization binding, phase order, verification, timeouts, localization
failure), crossing geometry, map validation, Dijkstra/A*, ordered waypoints,
localization-driven execution, telemetry fields, calibration maths, and CLI
refusal paths.

Commands and results are recorded in
`docs/agent-work/physical-navigation/PHASE2_REPORT.md` (phase 1 evidence remains
in `PHASE1_REPORT.md`).


## 14. Version 0.2.0 follower and safety verification

The follower estimates the selected oriented boundary, not the centroid of
black readings. Centered BLACK_LEFT patterns 1100/0100 and BLACK_RIGHT
0011/0010 produce zero correction. It acquires while stopped, filters the
PD derivative, limits correction slew, reduces speed around junction patterns,
rejects stale/future/replayed evidence and bounds recovery. The first-run
configuration disables recovery and caps wheels at 5 PWM counts (base 4).

Curve handling reduces the forward component in proportion to filtered steering
demand (`curve_slowdown_start`, `min_curve_speed_factor`). This gives the
differential correction more yaw authority without raising the wheel PWM cap.
`config/nav.windy-first-run.json` is the bounded 5-count profile for physical
curve diagnosis. Its synthetic alternating-turn regression covers both boundary
orientations and includes the older fixed-speed profile as a failure control.
This remains reactive: four binary sensors cannot see a turn before it reaches
the sensor bar.

Both `waferbot follow` and `scripts/test_line_follow.py --physical` use the same
CLI session: verified mapping gate, interactive consent, exclusive process
ownership, shared stop latch, synchronized motor writes, watchdog and cleanup.
An external stop is tested during commanded movement over an injected transport.
Software stops still cannot replace a physical power cutoff.

The new tape simulator computes sensor bits from the moving chassis pose and
finite-width tape. Both boundaries, offsets, gentle curves, noise/dropouts,
sensor failure and wrong-feedback-sign controls are tested. Its geometry,
friction-free kinematics and gains are synthetic; no physical tracking accuracy
is claimed. The digital array has a dead band between its middle sensors.

Switching has a total moving-time budget (`max_travel_s`) and, when physical,
a measured distance-derived budget across crossing/search/resume. Stopped
prompts do not spend moving time; sensor evidence is reacquired after prompts.
Absolute map headings are CCW from +x. Reverse FOLLOW is refused before motion,
rather than applying a forward controller to a backwards-facing sensor array.
Manual TURN confirmation includes the expected heading. Real marker integrations
must establish the robot's heading as well as node identity on the actual track.

Version 0.2.0 verification evidence and the requirements-to-files mapping are in
`docs/agent-work/physical-navigation/FINAL_REPORT.md`. The complete source file
inventory is `docs/PHYSICAL_NAVIGATION_FILES.md`.

## 15. Stationary AprilTag recognition

The repository had no prior AprilTag reader. Vendored Yahboom demos use OpenCV
`VideoCapture(0)` for a USB camera and reference an external stock OLED driver
at `/home/pi/software/oled_yahboom/yahboom_oled.py`. The new
`waferbot.vision.apriltag` module uses OpenCV's `DICT_APRILTAG_36h11` detector
and the driver's `Yahboom_OLED` interface. It records numeric IDs, UTC CSV
events and image corners, but does not estimate a physical pose or infer a graph
node. Its string `marker_id` can later be matched to a measured map node.

`waferbot tags` opens only the camera and display. It neither constructs a
`Robot` nor opens the motor controller, and camera/display cleanup runs on
exit. The notice returns to this command's normal OLED screen after a short
interval. Because the external OLED driver has no prior-screen readback, the
normal lines are configurable; do not run another OLED writer concurrently.
OpenCV is an optional vision dependency, installed separately from the base
offline motion bundle. See `docs/APRILTAG_TESTING.md` for the exact Pi commands.
