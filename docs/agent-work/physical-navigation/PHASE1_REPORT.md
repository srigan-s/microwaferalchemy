# Phase 1 Report: Verified Hardware Foundation

## STATUS

`ready_for_review`

Task: `/root/physical_navigation` (phase 1 only)
Workspace: `/Users/srigan/microwaferalchemy`
Writer: one implementation worker. No subagents, no orchestration skill, no
commits, no pushes, no live Pi, no hardware or environment changes.

Phase 1 delivered the verified hardware foundation; controllers, planner,
executor, CLI, telemetry, and deployment are untouched and deferred to phase 2 as
instructed.

## Changed paths

New files only. No baseline file was modified; the four vendor sources that carry
the protocol evidence still hash-match `baseline.json` (checked below).

| Path | Behaviour |
| --- | --- |
| `pyproject.toml` | `waferbot` package (src layout, Python >= 3.11); extras `hardware` = `smbus2`, `test` = `pytest`; pytest config |
| `src/waferbot/protocol.py` | Leaf constants: address `0x2B`, bus 1, register map, direction/speed limits |
| `src/waferbot/errors.py` | `WaferbotError`, `I2CError`, `MotorCommandError`, `SensorError`, `SafetyError`, `SpeedLimitError`, `ConfigError` |
| `src/waferbot/config.py` | `MotorConfig`, `SensorConfig`, `SafetyConfig`, `RobotConfig` with strict validation and JSON round-trip |
| `src/waferbot/kinematics.py` | `DriveAction`, `WHEEL_SIGNATURES`, `wheel_speeds()` from the vendored mixing formula |
| `src/waferbot/safety.py` | `SafetyState`, `FaultCode`, `FaultRecord`, arming, latched emergency stop, motion deadline watchdog |
| `src/waferbot/robot.py` | `Robot` facade: `forward`/`backward`/`strafe_left`/`strafe_right`/`rotate_left`/`rotate_right`/`stop`, `arm`/`disarm`, fault and e-stop handling, `tick()` |
| `src/waferbot/hardware/transport.py` | `I2CTransport` protocol; `Smbus2Transport` (lazy `smbus2` import, every failure raises `I2CError`) |
| `src/waferbot/hardware/registers.py` | `motor_block`, `stop_block`, `motor_blocks` and constant re-exports |
| `src/waferbot/hardware/motor_driver.py` | Four-wheel writes, bounded values, best-effort all-wheel stop on partial failure |
| `src/waferbot/hardware/sensors.py` | `LineReading`, `LineSensorArray`, `decode_line_byte` with configurable polarity and bit mapping |
| `src/waferbot/hardware/mock.py` | `MockI2CTransport` with recording and fault injection; `build_mock_robot()` |
| `tests/conftest.py`, `tests/test_protocol.py`, `tests/test_sensors.py`, `tests/test_safety.py`, `tests/test_motor_faults.py`, `tests/test_config.py`, `tests/test_import_safety.py` | 64 hardware-free tests |
| `HARDWARE_IMPLEMENTATION.md` | Protocol, wheel-mapping, and polarity evidence; interfaces, safety contract, configuration, hardware confirmation checklist |
| `docs/agent-work/physical-navigation/PHASE1_REPORT.md` | This report |

## Behaviour highlights

* Import, transport construction, driver construction, and `Robot` construction
  emit zero I2C writes: nothing can move at boot or at object creation.
* Motion requires `arm()`; movement is refused while disarmed, faulted, or after
  an emergency stop. Speed is bounded by `MotorConfig.max_speed` (default 100) and
  by the protocol limit 255; violations raise before any bus traffic.
* I2C failures are never printed and swallowed. A mid-command failure attempts a
  stop for all four wheels and then raises `MotorCommandError` with the original
  `I2CError` chained.
* Every motion command refreshes a deadline; `robot.tick()` stops all wheels when
  it expires, so a stalled control loop cannot drive indefinitely.
* Sensor reads normalize to `BLACK = 1`, `WHITE = 0` with polarity and channel
  order configurable, and malformed frames raise `SensorError` instead of
  returning a default.

## Verification

All commands run from `/Users/srigan/microwaferalchemy`. The interpreter is a
throwaway venv outside the repository (`/tmp/waferbot-phase1-venv`, Python
3.11.16) so no environment or repository files were added.

| # | Command | Exit | Salient result |
| --- | --- | --- | --- |
| 1 | `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python -m pytest -p no:cacheprovider` | 0 | `64 passed in 0.05s` (config 9, import-safety 3, motor-faults 6, protocol 18, safety 13, sensors 15) |
| 2 | `python -m compileall -q src tests` | 0 | Every module and test compiles; `__pycache__` artifacts removed afterwards |
| 3 | `pip wheel . -w <tmp> --no-deps` (from a copy of `pyproject.toml`, `src`, `tests`, `HARDWARE_IMPLEMENTATION.md`) | 0 | `Successfully built waferbot`; `waferbot-0.1.0-py3-none-any.whl` (28,896 bytes) |
| 4 | Install that wheel, then `python -c "import waferbot, sys; print('version', waferbot.__version__); print('smbus2 imported:', 'smbus2' in sys.modules)"` | 0 | `version 0.1.0` / `module /tmp/.../site-packages/waferbot/__init__.py` / `smbus2 imported: False` |
| 5 | Re-run the suite inside the build copy against the installed wheel | 0 | `64 passed in 0.06s` - packaging, not just the source tree, is verified |
| 6 | Mocked dry run of the public API (no hardware) | 0 | `writes after construction: 0`; unarmed `forward(40)` refused with `motion refused: robot is not armed`; `forward(40) -> [(0,0,40),(1,0,40),(2,0,40),(3,0,40)]`; `strafe_left(40) -> [(0,1,40),(1,0,40),(2,0,40),(3,1,40)]`; `rotate_right(40) -> [(0,0,40),(1,0,40),(2,1,40),(3,1,40)]`; overspeed `300` refused with zero writes; `emergency_stop` produced four stop blocks and state `EMERGENCY_STOP` |
| 7 | Baseline hash check of `raspbot/Raspbot_Lib.py`, `lib/McLumk_Wheel_Sports.py`, `7.Status of four-way line patrol module.ipynb`, `1.infrared_patrol_line.ipynb` against `baseline.json` | 0 | All four `OK` (byte-identical) |
| 8 | Import and drive the package under the host's Python 3.14.7 as well | 0 | `python 3.14.7 \| writes: 8` (4 forward + 4 stop) - the package is not tied to a single interpreter |

Test assertions of note (all against mocks, no hardware):

* Exact bytes and motor-id order for all six primitives, plus `stop`.
* Rotation and strafe patterns are asserted to differ, and rotation is asserted
  to be differential (both left wheels share a sign).
* Partial write failure at the third wheel yields exactly
  `[(0,0,60),(1,0,60)]` then four stop blocks, with `MotorCommandError` and a
  chained `I2CError`.
* `stop_all` attempts all four wheels even when every write fails, and re-raises
  the first error.
* Polarity, the `(2, 3, 1, 0)` bit map, malformed/empty frames, and read failures.
* Arming refusal without writes, e-stop latch/clear (confirm required, no
  auto-rearm), fault latch blocking arming, watchdog expiry stopping the wheels,
  and deadline refresh on repeated commands.

## Evidence for the protocol claims

Summarized in `HARDWARE_IMPLEMENTATION.md` sections 4-7 and 9; the strongest
citations are `Raspbot_Lib.py:7,34-40,49-55,58-88` (address `0x2B`, block writes,
register `0x01`, `[id, dir, speed]`), `McLumk_Wheel_Sports.py:149-163`
(`set_deflection`), `McLumk_Wheel_Sports.py:74-92` (rotation overrides),
`1.infrared_patrol_line.ipynb:33-79` (channel roles and `0 = black`), and
`7.Status of four-way line patrol module.ipynb:55-70` (bit extraction and
`x2 x1 x3 x4` display order).

## Stable interfaces chosen

Worth locking before phase 2 builds on them:

* `Robot.forward/backward/strafe_left/strafe_right/rotate_left/rotate_right(speed)`
  return the four signed wheel speeds they issued; `speed` is a PWM magnitude in
  counts, never m/s.
* `Robot.stop()` is always permitted; `Robot.tick()` returns `True` when the
  watchdog fired.
* `Robot.arm()`, `disarm()`, `raise_fault(FaultCode, message)`, `clear_fault()`,
  `emergency_stop(...)`, `clear_emergency_stop(confirm=True)`, `robot.state`.
* `LineSensorArray.read() -> LineReading(raw_byte, raw(4), normalized(4),
  timestamp)` with `raw`/`normalized` ordered S1..S4.
* `I2CTransport.write_block(address, register, data)` /
  `read_block(address, register, length)` as the single injectable seam.
* `RobotConfig` JSON with `motor` / `sensor` / `safety` sections, strict
  validation, unknown keys rejected.

## Outstanding risks and hardware uncertainties

1. **Sensor bit-to-channel order is inferred, not measured.** The
   `(2, 3, 1, 0)` mapping comes from one follower notebook plus one display
   string. Polarity (`0 = black`) is much better supported. Confirm both with the
   phase 2 `waferbot sensors` diagnostic and adjust `SensorConfig` if needed;
   see `HARDWARE_IMPLEMENTATION.md` section 7.2.
2. **`L1` vs `L2` front/rear is unverified.** No vendored file states it. It does
   not affect any phase 1 primitive (all use left-pair vs right-pair grouping),
   but it must be confirmed before diagonal moves are added. Held in
   `MotorConfig.wheel_labels`.
3. **No hardware test has been executed.** All 64 tests use mocks. Nothing here
   proves the board responds as the vendored source implies.
4. **No independent hardware watchdog is verified.** A killed process, a wedged
   bus, or power loss can leave the board holding its last command; the software
   deadline only helps while something still calls `tick()`. Physical motor
   power cutoff remains an operator requirement.
5. **No speed calibration.** Counts are not converted to m/s anywhere, which is
   deliberate; `speed_limit_mps` semantics in the phase 2 map will need either a
   calibration table or an explicit counts-based interpretation.
6. **`smbus2` is not installed in this sandbox**, so `Smbus2Transport`'s happy
   path is untested here; only its missing-dependency path is exercised (test 3
   of `tests/test_import_safety.py`). The Pi install must confirm real transfers.

## Decisions requiring Astra

1. Confirm the phase 1 interface freeze (movement signatures, `tick()`
   watchdog ownership, `LineReading` shape) before phase 2 codes against it.
2. Decide whether phase 2 should expose a counts-based speed or require a
   measured calibration before any `speed_limit_mps` field becomes meaningful.
3. Decide whether phase 2 ships the `waferbot sensors` / `waferbot motor-test`
   diagnostics as the mandatory first hardware step (recommended: yes, before any
   floor test), given the two open mapping uncertainties.

## Next checkpoint

Phase 2 (after this review): edge detection and debouncing (`detect_edge`),
`EdgeFollower`, edge-switch FSM with bounded recovery and confirmation, crossing
angle geometry, JSON track map with validation, Dijkstra/A* with mandatory
waypoints, `RouteExecutor` with marker-localization interfaces, `waferbot` CLI
(mock default, explicit physical selection, arming prompt), CSV telemetry,
deployment scripts, and mocked end-to-end route tests - all reusing the frozen
interfaces above.
