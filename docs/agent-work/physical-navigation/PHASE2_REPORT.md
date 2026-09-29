# Phase 2 Report: Full physical navigation stack

## STATUS

`ready_for_review`

Task: `/root/physical_navigation` (phase 2, full build)
Workspace: `/Users/srigan/microwaferalchemy`
Writer: one implementation worker. No subagents, no orchestration skill, no
commits, no pushes, no deployment execution, no live hardware, no system
changes.

## Mandatory safety corrections (applied first, regression tests written first)

`tests/test_safety_regressions.py` (21 cases) was added before the code changes
and drives all of the following:

1. `Robot.disarm()` now stops the wheels (not just the deadline) and re-raises a
   latched `MOTOR_COMMUNICATION_FAILURE` if that stop fails.
2. `Robot.drive()`/`drive_wheels()` failures latch
   `FaultCode.MOTOR_COMMUNICATION_FAILURE`, attempt a stop for every wheel, and
   re-raise with the original `I2CError` preserved as `__cause__`.
3. Line-sensor read failures (bus error **or** malformed frame) latch
   `FaultCode.SENSOR_FAILURE`, stop the wheels, and re-raise.
4. A failed `stop()`, `disarm()`, fault-handling stop, or emergency-stop stop
   leaves the robot disarmed and recorded (`note_failure` extends the existing
   fault message instead of hiding the earlier fault).
5. `MotionWatchdog` runs an independent daemon thread that stops the wheels and
   latches `FaultCode.MOTION_TIMEOUT` even when the controller never calls
   `tick()` again.
6. Every bus access is serialised on one re-entrant `BusLock`; a test proves a
   watchdog stop cannot split a four-wheel command (the in-flight command
   finishes, then the stop lands).
7. Fractional and boolean wheel counts are rejected (`MovementCommandError`-free
   `ValueError`/`SpeedLimitError`) instead of being `int()`-coerced.
8. `motor.bus` and `sensor.bus` must match; a mismatch raises `ConfigError`.

Additional phase 2 gates from the checkpoint:

* Physical floor commands require `motor.verified` and `sensor.verified` in
  configuration (or the explicit `--ack-verified-config` acknowledgement).
* Physical graph execution requires a measured `nav.counts_to_mps`; each edge's
  `speed_limit_mps` is then converted to a PWM cap for that step.

## What was implemented (REQUEST sections 3-15)

| Section | Delivered |
| --- | --- |
| 3 | `sensing/edge.py`: `EdgeState`, `detect_edge`, `EdgeDetector`, `edge_byte_for`; polarity, channel mapping, freshness-aware debounce, stale data resets stability, outer sensors, junction flag |
| 4 | `sensing/follower.py`: `EdgeFollower` at configurable 20 Hz with PD control, differential/lateral/blended correction, junction slowdown, bounded recovery, `LINE_LOST` latch, `finally` stop, optional arrival monitor |
| 5 | `sensing/switching.py`: `EdgeSwitcher` FSM (FOLLOW_EDGE → APPROACH → CONFIRM → REDUCE_SPEED → EXECUTE → SEARCH → VERIFY → RESUME), authorization binding source/target/location, lateral + diagonal modes, bounded timeouts, monotonic fresh confirmation, destination confirmation |
| 6 | `geometry.py`: minimum-angle formula, `evaluate_crossing` with sensor footprint, sampling distance, forward clearance, robot clearance; `calibration/crossing.py` sweep with success rate/time/final edge CSV |
| 7 | `nav/graph.py`: adjacency-list JSON map with node/edge fields, strict validation, enabled edges, positive/negative → edge-state mapping that must be measured, physical-readiness gate, `maps/example_track.json` |
| 8 | `nav/planner.py`: Dijkstra (default) and A*, cost = travel + switch + dock + turn overhead, admissible heuristic with metric-consistency check (otherwise zero), ordered mandatory waypoints, unreachable/unknown handling |
| 9 | `nav/executor.py`: `RouteExecutor` verifying start location, following edges, confirming arrival by localization (never by time), authorized switching, TURN/DOCK bounded moves with confirmation, dry run |
| 10 | `safety.py` + `robot.py` + `watchdog.py` + `process.py`: states, typed faults, e-stop, obstacle hook (`hardware/ultrasonic.py`), timeouts, Ctrl+C handling, mock mode, best-effort stops |
| 11 | `cli.py`: `sensors`, `motor-test`, `calibrate wheels|sensors|crossing|speed`, `follow`, `switch`, `map`, `plan`, `execute`, `stop`; mock default, explicit physical prompt, configurable speed, dry run |
| 12 | `deployment/{install_pi,deploy,verify_hardware,offline_install}.sh` + `scripts/build_offline_bundle.py` + actual bundle in `dist/waferbot-offline-0.1.0/` |
| 13 | Hardware-free tests (296 after phase 2, **393** after the correction bundle); `telemetry.py` CSV with sensor/motor/control/localization/fault fields |
| 14 | No ROS 2/Gazebo/simulation exists in this repository: the same graph, planner, and executor run against the mock, documented in the README |
| 15 | `README.md` runbook (SD-card flashing, install, first sensor/motor/follow/switch commands, A+ → B- → D-, deployment), this report, updated `HARDWARE_IMPLEMENTATION.md` |

## Stable interfaces and decisions

* Movement: `Robot.forward/backward/strafe_left/strafe_right/rotate_left/rotate_right(speed)`,
  `Robot.drive_wheels(speeds)` (per-wheel signed counts), `Robot.stop()`,
  `Robot.tick()`, all returning the issued per-wheel speeds.
* Safety: `arm()`, `disarm()`, `raise_fault(code, message)`, `clear_fault()`,
  `emergency_stop(...)`, `clear_emergency_stop(confirm=True)`, `state`,
  `FaultCode` (adds `MOTION_TIMEOUT` alongside the six requested codes).
* Sensing: `LineReading(raw_byte, raw, normalized, timestamp)` and
  `EdgeDetection(raw, normalized, edge, timestamp, stable, stable_edge,
  stable_count, age_s, outer_left, outer_right, junction)`.
* Navigation: `plan_route(track, start, goal, required_waypoints=[],
  algorithm=...) -> Route` and `RouteExecutor.execute_route(route, dry_run=...)`.
* Localization: `ArrivalMonitor.check(expected_node) -> Localization | None`
  with `requires_stop()`; mock, scripted, and manual monitors provided.
* PWM counts are exposed for follow/diagnostics; m/s is only used where a
  measured `counts_to_mps` exists, and never invented.

## New and modified files

Phase 1 files were extended (not replaced): `config.py`, `robot.py`,
`hardware/motor_driver.py`, `hardware/sensors.py`, `hardware/mock.py`,
`hardware/__init__.py`, `sensing/*`, `errors.py`, `pyproject.toml`
(`waferbot` console script), `README.md`, `HARDWARE_IMPLEMENTATION.md`.

| Path | Lines |
| --- | --- |
| `README.md` | 299 |
| `HARDWARE_IMPLEMENTATION.md` | 462 |
| `pyproject.toml` | 30 |
| `maps/example_track.json` | 75 |
| `config/robot.example.json` | 22 |
| `config/nav.example.json` | 66 |
| `deployment/install_pi.sh` | 171 |
| `deployment/offline_install.sh` | 91 |
| `deployment/verify_hardware.sh` | 83 |
| `deployment/deploy.sh` | 115 |
| `deployment/README.md` | 50 |
| `scripts/build_offline_bundle.py` | 288 |
| `src/waferbot/__init__.py` | 64 |
| `src/waferbot/protocol.py` | 74 |
| `src/waferbot/errors.py` | 66 |
| `src/waferbot/config.py` | 281 |
| `src/waferbot/navconfig.py` | 370 |
| `src/waferbot/kinematics.py` | 100 |
| `src/waferbot/safety.py` | 261 |
| `src/waferbot/watchdog.py` | 114 |
| `src/waferbot/process.py` | 263 |
| `src/waferbot/localization.py` | 244 |
| `src/waferbot/geometry.py` | 239 |
| `src/waferbot/telemetry.py` | 233 |
| `src/waferbot/robot.py` | 418 |
| `src/waferbot/cli.py` | 967 |
| `src/waferbot/hardware/bus.py` | 47 |
| `src/waferbot/hardware/motor_driver.py` | 153 |
| `src/waferbot/hardware/sensors.py` | 145 |
| `src/waferbot/hardware/registers.py` | 104 |
| `src/waferbot/hardware/transport.py` | 104 |
| `src/waferbot/hardware/ultrasonic.py` | 141 |
| `src/waferbot/hardware/mock.py` | 176 |
| `src/waferbot/sensing/edge.py` | 289 |
| `src/waferbot/sensing/follower.py` | 423 |
| `src/waferbot/sensing/switching.py` | 666 |
| `src/waferbot/nav/graph.py` | 553 |
| `src/waferbot/nav/planner.py` | 342 |
| `src/waferbot/nav/executor.py` | 646 |
| `src/waferbot/calibration/wheels.py` | 184 |
| `src/waferbot/calibration/sensors.py` | 160 |
| `src/waferbot/calibration/crossing.py` | 161 |
| `src/waferbot/calibration/speed.py` | 106 |
| `tests/` (21 test files + `conftest.py` at phase 2; see the correction section for the final 27-file manifest) | 3,904 |
| `dist/waferbot-offline-0.1.0/` | generated bundle (app + 2 wheels + manifest) |
| `docs/agent-work/physical-navigation/PHASE1_REPORT.md` | 100 (phase 1) |
| `docs/agent-work/physical-navigation/PHASE2_REPORT.md` | this file |
| `docs/agent-work/physical-navigation/logs/*.log` | full verification logs |

Test counts per file: calibration 11, cli 20, config 12, deployment_assets 13,
edge_detection 12, executor 16, follower 14, geometry 14, graph 26,
import_safety 3, motor_faults 6, navconfig 32, planner 12, process 5,
protocol 18, safety 13, safety_regressions 21, sensors 15, switching 19,
telemetry 6, ultrasonic 8 — **296 total** at phase 2. The correction bundle below
adds six files and raises the total to **393**; its section carries the exact
per-file manifest for the final state.

## Verification (full logs in `logs/phase2-verification.log` and
`logs/baseline-check.log`)

| # | Command | Exit | Result |
| --- | --- | --- | --- |
| 1 | `PYTHONPATH=src python -m pytest -p no:cacheprovider` | 0 | `296 passed in 0.76s` (phase 2 snapshot; 393 in the correction bundle) |
| 2 | `python -m compileall -q src tests scripts` | 0 | every module compiles |
| 3 | `bash -n deployment/*.sh` | 0 | all four scripts parse |
| 4 | `python scripts/build_offline_bundle.py --output-dir dist` | 0 | bundle built: `waferbot-0.1.0` + `smbus2-0.6.1` wheels, 49-file manifest, no vendor/credential paths |
| 5 | `bash deployment/offline_install.sh --bundle dist/waferbot-offline-0.1.0 --venv <clean>` | 0 | hash verification of 49 files, `--no-index` install of `waferbot[hardware]` in a fresh venv |
| 6 | installed CLI: `sensors`, `map`, `plan`, `execute` (mock), `follow`, `switch`, `motor-test`, `stop` | 0 | sensor table `0x07`; map 21 nodes/28 edges, `physical_ready False`; plan `A+ → B+ → B1 → B2 → B3 → B- → C- → D-`; mock `execute` completed with 7 steps ending at `D-`; 51 telemetry rows across sensor/motor/follow/localization events; unauthorized `switch` refused (3); motion while a stop latch is set refused (3) |
| 7 | baseline all-files hash check against `baseline.json` | 0 | `files checked: 76997`, `missing: 0`, `mismatched: 0`, `OK (baseline byte-identical)` |

## Outstanding risks and limitations

1. **No hardware has been exercised.** Every test uses the mock transport; the
   `0x2B` protocol, wheel mapping, sensor polarity, and ultrasonic frame are
   derived from vendored source, not measured here.
2. **Sensor bit-to-channel order is inferred.** Polarity (`0 = black`) has strong
   in-line evidence; the S1..S4 → bit mapping comes from one follower notebook
   plus one display string. `waferbot calibrate sensors` exists to confirm or
   correct it.
3. **Front/rear wheel assignment is unverified.** Assumed 0 = front-left,
   1 = rear-left, 2 = front-right, 3 = rear-right. It does not affect any phase 1
   or phase 2 primitive (all use left-pair/right-pair grouping) but it is the
   label set the calibration report prints.
4. **Time-bounded manoeuvres without encoders.** TURN/DOCK and the switch
   crossing are bounded by time; correctness depends on the operator-supplied
   durations and the measured speed factor. There is no odometry.
5. **No hardware watchdog.** `SIGKILL`, a wedged bus, or power loss can leave the
   controller holding its last command; the software watchdog and stop latch only
   help while a process is alive. Physical power cutoff remains mandatory.
6. **`smbus2` happy path untested here.** The dependency is bundled and installs
   offline, but no real transfer has been attempted from this machine.
7. **`deploy.sh` and the Pi installers were syntax-checked, not executed.** No
   SSH target or Pi was touched, per the task rules.
8. **Raspberry Pi documentation pages returned HTTP 403 to automated fetches**
   from this machine; the Imager/SSH/headless and I2C wording in `README.md` was
   checked against the official `raspberrypi/documentation` repository content
   instead (commit `34dfb87`).

## Decisions requiring Astra

1. Confirm the phase 2 interface set (`EdgeDetection`, `Route`, `ArrivalMonitor`,
   `SwitchAuthorization`) before any further work builds on it.
2. Confirm the policy that physical execution refuses to start without
   `counts_to_mps`, `verified` flags, and a physical-ready map (the strictest
   reading of the checkpoint instruction).
3. Decide whether TURN/DOCK should stay time-bounded or wait for a measured
   distance source (encoders/fiducials) before physical use.

## Next checkpoint

Root review of this patch, then any correction cycle. No further implementation
is in flight; the workspace is left with a green test suite, a built offline
bundle in `dist/`, and an untouched vendor tree. The correction bundle below
records the review response and the final numbers.

---

# Correction bundle (response to `REVIEW.md`)

## STATUS

`ready_for_review` — all review sections addressed, 393 tests green, bundle
rebuilt, baseline verified byte-identical. No commits, pushes, deployments, SSH,
apt, or hardware access.

## Review section → fix

| Review | Fix | Evidence |
| --- | --- | --- |
| 1 stop/motion atomicity | `Robot._transaction` holds the in-process lock **and** the cross-process bus guard around the external-stop check, arming check, deadline refresh, and write. `ProcessGuard.bus_guard` is re-entrant per thread so nested stop paths cannot deadlock. | `tests/test_atomicity.py` (e-stop mid-command wins; external stop under the transaction; bus-lock timeout latches `MOTOR_COMMUNICATION_FAILURE`; nested guard has no deadlock) |
| 1 `tick` cancels watchdog | `tick()` only acts on a real expiry and then latches `MOTION_TIMEOUT`; `SafetyController.enforce_deadline` latches as well. | `test_tick_does_not_cancel_a_live_deadline`, `test_tick_expiry_latches_motion_timeout` |
| 1 `waferbot stop` | `stop --physical` takes the bus lock and writes the four stop blocks itself, reports `stop_bytes_written`/`bus_lock_acquired`/error, refuses `--clear` while a live session owns the bus, and the transport is closed in every lifecycle path. | `test_physical_stop_writes_stop_bytes`, `test_clear_is_refused_while_a_session_owns_the_bus`, `test_transport_is_closed_on_lifecycle_cleanup` |
| 2 fabricated localization | `--localizer` default is `auto`; physical `scripted` is refused **before any bus work**; physical runs use stopped operator prompts or a real `--marker-reader`. `cmd_switch`/crossing sweep no longer use fixed/mock providers on hardware; each sweep attempt stops, confirms repositioning, authorizes, and records failures. | `tests/test_cli_physical.py` (scripted refused with zero transports created; manual localization completes; obstacle monitor integration) |
| 3 fresh evidence | `EdgeDetector` uses the observation timestamp: nonfinite/future/nonincreasing/gapped/replayed frames reset stability, channels are validated, and `edge_byte_for` honours inverted polarity. | `test_replayed_reading_never_becomes_stable`, `test_backwards_and_gapped_evidence_resets_stability`, `test_edge_byte_for_respects_configured_polarity` |
| 3 truthful arrival | `LocalizationPolicy`/`LocalizationConfirmer` centralise identity, finite confidence, freshness, and strictly-newer evidence checks at every confirmation point; `Localization.matches` no longer accepts a matching marker when `node_id` names another node; the executor adopts the localizer's real object instead of synthesising `source="arrival"`. | `tests/test_localization.py`, `test_physical_execute_uses_manual_localization` (final source is `manual`) |
| 4 motion loops/bounds | `Robot.run_command` refreshes any timed hold faster than the watchdog and bounds it by real time; wheel probes, TURN/DOCK, and recovery use it. Ambiguous/wrong-edge observations are bounded by `max_ambiguous_s`/`max_wrong_edge_s`; recovery needs *stable* target readings; NaN/inf/negative durations and bad counts are rejected before motor writes; PWM fields must be integers. | `test_wheel_probe_survives_an_enabled_watchdog`, `test_recovery_ignores_a_flash_of_the_target_edge`, `test_persistent_ambiguity_is_bounded`, `test_follow_rejects_invalid_durations_before_moving` |
| 5 switch geometry/travel | `max_travel_s` and a measured `switch.max_travel_m` form one shared budget for crossing+search+resume; physical switches require `counts_to_mps`; every moving phase re-checks arming/faults; diagonal blends are scaled to the calibrated cap; sub-one-count ceilings are refused; per-edge `crossing_angle_deg` (90 = lateral) drives each switch; speed caps apply to TURN/DOCK; TURN executes the signed heading change and reverse, FOLLOW refuses reverse. | `tests/test_route_and_cleanup.py`, `test_physical_switch_requires_a_measured_travel_budget`, `test_diagonal_edge_requires_measured_geometry`, `test_diagonal_blend_never_exceeds_its_ceiling` |
| 5 geometry | Added measured `lateral_clearance_m` and `required_confirmations`; observability uses lateral speed ÷ real polling rate × confirmations + detection width; the bogus robot-width-vs-longitudinal comparison is replaced by a lateral clearance check; physical evaluation uses `CrossingInputs.from_measured_command` (commanded counts × factor, real rate). | `tests/test_geometry.py` (`test_missing_lateral_clearance_is_flagged`, `test_observability_uses_lateral_speed_rate_and_confirmations`, `test_inputs_can_be_built_from_the_commanded_values`) |
| 6 route/map/planner | `_validate_route` checks start/goal membership, node-sequence continuity, step continuity, membership in the *current* map, stale values, enabled state, direction support, edge sides, geometry, and ordered waypoints before motion; STOP must be stationary/self and non-coincident STOP maps are rejected; the planner no longer skips STOP in metric consistency and no longer double-counts `g`; `TrackMap` uses a real adjacency index. | `tests/test_route_and_cleanup.py` (forged/stale/disabled/sequence/waypoint/reverse/spatial-STOP cases, A* optimality, adjacency) |
| 7 cleanup truth | Follower/switch/executor perform the mandatory stop *before* reporting success; a failed final stop marks the result failed; `_fail` appends to an existing fault instead of relabelling (a sensor failure stays `SENSOR_FAILURE`). | `test_follower_reports_fault_when_the_final_stop_fails`, `test_switcher_reports_failure_when_the_final_stop_fails`, `test_sensor_failure_is_not_relabelled_as_switch_timeout` |
| 8 calibration | `compute_counts_to_mps` returns `max(ratios)/conservatism` (an upper bound, so derived caps stay under the limit) with finite-positive validation; sensor calibration measures an all-white baseline, requires a consistent single-bit change per channel, and infers either polarity; wheel `confirmed` needs all four wheels and all direction checks; sweep CSV keeps raw rows plus per-angle summaries and never recommends an angle when every attempt failed; instructions rewritten. | `tests/test_calibration.py` (inverted polarity, unstable baseline, multi-bit rejection, partial-wheel run, derived cap bound, all-failed sweep) |
| 9 deployment | Bundle now ships `deployment/` at its root (and inside `app/`), writes `README.txt` before the manifest, records pinned wheels with hashes and licence metadata, verifies required wheels, and `--no-download` reuses existing wheels instead of deleting them. `deploy.sh` uses the correct non-doubled paths, quotes/validates the custom venv, expands `$HOME` on the Pi deliberately, and drops `--info=progress2`. `offline_install.sh` never upgrades pip and installs `--no-index` only; `install_pi.sh --bundle` is offline too and now checks the I2C group and `/dev/i2c-1`. `verify_hardware.sh` propagates `--bus/--address` through a temporary config and uses the read-only sensor lifecycle. | `tests/test_deploy_shim.py` (ssh/rsync shims execute the generated remote commands in a fake Pi layout), `test_offline_install_never_upgrades_pip`, `test_install_pi_bundle_mode_never_upgrades_pip_online`, `test_shipped_offline_installer_works_with_no_network` |
| 9 CLI/hardware-independent | `sensors`/`motor-test`/`calibrate` no longer load a map, `map`/`plan` resolve the packaged `waferbot/data/example_track.json`, `sensors --physical` is a read-only lifecycle (no stop writes), `execute --dry-run` never constructs a transport, and session construction cleans up telemetry/ownership/transport on failure. | `test_sensors_work_without_a_map_from_another_directory`, `test_map_command_resolves_the_packaged_example`, `test_physical_sensors_is_truly_read_only`, `test_physical_dry_run_never_opens_the_transport` |
| 10 obstacle/telemetry | `--obstacle-distance-mm` wires the ultrasonic monitor into physical sessions (off by default) and it can trigger `OBSTACLE_DETECTED`/`SENSOR_FAILURE`; telemetry logs stops, faults, switch phases, and switch results, the follower no longer reports the expected destination as `current_node`, a failed crossing never claims the target edge was observed, and the logger is lock-protected, idempotent on close, and drop-safe after close. | `tests/test_telemetry.py`, `test_obstacle_monitor_runs_for_physical_sessions` |

## Correction verification

Full log: `docs/agent-work/physical-navigation/logs/correction-verification.log`.

| # | Command | Exit | Result |
| --- | --- | --- | --- |
| 1 | `PYTHONPATH=src python -m pytest -p no:cacheprovider` | 0 | **393 passed** |
| 2 | `python -m compileall -q src tests scripts` | 0 | every module compiles |
| 3 | `bash -n deployment/*.sh` | 0 | all four scripts parse |
| 4 | `python scripts/build_offline_bundle.py --output-dir dist` | 0 | bundle rebuilt: 56 manifest files, `README.txt` included, `deployment/` at the root, `waferbot-0.1.0` + `smbus2-0.6.1` wheels with hashes/licences, no vendor or credential paths |
| 5 | `python scripts/build_offline_bundle.py --no-download` | 0 | existing wheels reused (not deleted); application files refreshed |
| 6 | `PIP_INDEX_URL=http://127.0.0.1:1 bash dist/.../deployment/offline_install.sh --bundle . --venv <clean>` | 0 | 56 hashes verified, required wheels present, `Successfully installed smbus2-0.6.1 waferbot-0.1.0` **with no reachable index** |
| 7 | installed CLI from `/tmp` | 0 | `waferbot map` resolves the packaged 21-node map; `sensors` prints frames; mock `execute A+ → B- → D-` completes at `D-` |
| 8 | baseline all-files hash check | 0 | `files checked: 76997`, `missing: 0`, `mismatched: 0` |

## Residual risks and hardware limits

1. No hardware has been exercised: protocol bytes, wheel mapping, sensor
   polarity, ultrasonic frames, and the stop path are still vendor-derived and
   mock-verified only. The injected-physical tests exercise the code paths, not
   the I2C bus.
2. Sensor bit-to-channel order still needs `waferbot calibrate sensors` on the
   real chassis; the calibration now measures a white baseline and requires a
   consistent single-bit change, but the operator must still cover one channel
   at a time correctly.
3. TURN/DOCK and switch crossings remain *timed* manoeuvres. They are bounded,
   refreshed, speed-capped, and confirmed by localization, but they contain no
   odometry and the distance budget is an explicit conservative estimate, never
   arrival evidence.
4. `counts_to_mps` from one straight run is not a general calibration; strafing,
   payload, and battery state change it. The derived cap is conservative under
   the *fastest measured* ratio only.
5. `SIGKILL`, a wedged bus, or power loss still defeat the software watchdog; a
   physical power cutoff remains mandatory. `waferbot stop --physical` needs a
   live process and a reachable bus.
6. `deploy.sh`/`install_pi.sh` were exercised through command shims and syntax
   checks; no real SSH session, apt install, raspi-config call, or Pi reboot has
   been performed.
7. The Raspberry Pi documentation pages still return HTTP 403 to automated
   fetches from this machine; the runbook wording was checked against the
   official `raspberrypi/documentation` repository instead.

## FILES manifest (exact)

Root docs: `README.md` (363), `HARDWARE_IMPLEMENTATION.md` (495),
`pyproject.toml` (33), `maps/example_track.json` (539),
`config/robot.example.json` (22), `config/nav.example.json` (72).

`src/waferbot/` (package `__init__` files included): `__init__.py` (64),
`protocol.py` (74), `errors.py` (66), `config.py` (281), `navconfig.py` (463),
`kinematics.py` (100), `safety.py` (266), `watchdog.py` (114), `process.py`
(300), `localization.py` (393), `geometry.py` (323), `telemetry.py` (261),
`robot.py` (542), `cli.py` (1248), `data/example_track.json` (539);
`hardware/__init__.py` (34), `hardware/bus.py` (47), `hardware/transport.py`
(104), `hardware/registers.py` (104), `hardware/motor_driver.py` (150),
`hardware/sensors.py` (145), `hardware/ultrasonic.py` (141),
`hardware/mock.py` (189); `sensing/__init__.py` (41), `sensing/edge.py` (340),
`sensing/follower.py` (493), `sensing/switching.py` (811);
`nav/__init__.py` (31), `nav/graph.py` (626), `nav/planner.py` (343),
`nav/executor.py` (874); `calibration/__init__.py` (38),
`calibration/wheels.py` (202), `calibration/sensors.py` (249),
`calibration/crossing.py` (177), `calibration/speed.py` (131).

`deployment/`: `install_pi.sh` (199), `deploy.sh` (146), `offline_install.sh`
(103), `verify_hardware.sh` (121), `README.md` (50).
`scripts/build_offline_bundle.py` (353).

Tests (27 files including `conftest.py`, 393 cases): `conftest.py` (70),
`test_atomicity.py` (236/11), `test_calibration.py` (307/18), `test_cli.py`
(304/20), `test_cli_physical.py` (313/10), `test_config.py` (103/12),
`test_deploy_shim.py` (267/7), `test_deployment_assets.py` (77/13),
`test_edge_detection.py` (190/17), `test_executor.py` (425/16),
`test_follower.py` (320/26), `test_geometry.py` (186/17), `test_graph.py`
(181/26), `test_import_safety.py` (45/3), `test_localization.py` (129/18),
`test_motor_faults.py` (92/6), `test_navconfig.py` (154/32),
`test_planner.py` (150/12), `test_process.py` (85/5), `test_protocol.py`
(131/18), `test_route_and_cleanup.py` (490/21), `test_safety.py` (160/13),
`test_safety_regressions.py` (368/21), `test_sensors.py` (114/15),
`test_switching.py` (502/19), `test_telemetry.py` (174/9),
`test_ultrasonic.py` (113/8).

Generated bundle `dist/waferbot-offline-0.1.0/`: `README.txt`, `bundle.json`
(wheel hashes + licence metadata), `manifest.json` (56 files),
`app/**` (allowlisted application files), `deployment/**` (installer copies),
`wheels/{waferbot-0.1.0,smbus2-0.6.1}-py3-none-any.whl` (contents and hashes
listed in `manifest.json`).

Logs: `logs/phase2-verification.log`, `logs/baseline-check.log`,
`logs/correction-verification.log`.

---

## Wrap-up status (user steering: stationary line-sensor test first)

Root now owns the standalone, no-motor diagnostic:
`scripts/test_line_edge.py`, `tests/test_line_edge_standalone.py`, and
`docs/LINE_EDGE_TEST.md`. This worker did not create or edit those files.

What is ready for that work right now (all verified on this machine):

* `waferbot sensors [--physical] [--samples N] [--interval S] [--json]` reads the
  four channels and prints raw byte, raw S1..S4, normalised BLACK=1/WHITE=0, and
  an ISO-ish timestamp. It needs no map, no nav config, and no calibration.
* `waferbot sensors --physical` is a read-only lifecycle: constructing the
  transport and Robot writes nothing, and cleanup closes the bus **without**
  sending stop blocks, so a sensor-only session cannot move the chassis.
* The same read path is available to a standalone script:
  `MockI2CTransport` (mock) or `Smbus2Transport` (Pi) plus `LineReading` /
  `decode_line_byte` / `EdgeDetector` / `detect_edge` from `waferbot.sensing`.
  `python -m pytest tests/test_sensors.py tests/test_edge_detection.py` covers
  polarity, channel mapping, debounce, stale/replayed frames, and malformed
  frames without hardware.
* Mock bytes for a specific edge are one call: `edge_byte_for(EdgeState.BLACK_LEFT)`
  → `0x07`, `edge_byte_for(EdgeState.BLACK_RIGHT)` → `0x0D`, with polarity
  respected.

Honest limitations of the navigation algorithms (not needed for the sensor
test, but relevant before any floor run):

1. **No hardware has ever been exercised.** Every result above is mock or
   injected-transport evidence. The I2C address, wheel mapping, sensor bit order,
   polarity, and ultrasonic frame are vendor-derived and still need the
   on-chassis confirmations in `HARDWARE_IMPLEMENTATION.md` section 12.
2. **Edge following is untested on tape.** The PD controller, junction slowdown,
   bounded recovery, and line-lost latch are unit-tested against scripted sensor
   sequences only; gains (`follow.kp/kd/base_speed`), the 20 Hz rate, and the
   recovery timings are untuned defaults.
3. **Edge switching is untested on tape.** The FSM, authorization binding,
   verification counts, travel budget, and destination confirmation are tested
   with scripted evidence; a real lateral/diagonal crossing has never been
   attempted. Diagonal crossing additionally needs measured geometry and
   `switch.max_travel_m`.
4. **No odometry.** TURN/DOCK and switch crossings are timed, bounded,
   watchdog-refreshed manoeuvres with localization confirmation; distances are
   explicit conservative estimates and never arrival evidence.
5. **Graph execution needs measured data.** Physical execution refuses an
   example/unvalidated map, unverified wheel/sensor assumptions, an uncalibrated
   `counts_to_mps`, and unmapped edge sides. The bundled map is illustrative
   only; `A+ → B- → D-` has only been executed in mock mode.
6. **Localization has no real reader yet.** Physical execution uses operator
   prompts (wheels stopped) unless you supply
   `--localizer marker --marker-reader module:attribute`.
7. **Software stops are not a power cut.** `SIGKILL`, a wedged bus, or power loss
   still defeat the watchdog; keep a physical cutoff within reach.

Final numbers: **393 tests passing**, `compileall` clean, all four deployment
scripts pass `bash -n`, the offline bundle rebuilt at
`dist/waferbot-offline-0.1.0/` (56 manifest files, wheels with hashes and licence
metadata, no vendor/credential paths), an offline install from the bundle's own
`deployment/offline_install.sh` succeeded with an unreachable package index, and
the vendor baseline is byte-identical (76,997 files, 0 missing, 0 mismatched).

Note for root: the bundle's `app/scripts/` is copied from the repository
`scripts/` directory, so if the standalone diagnostic is added after this
rebuild, re-run
`python scripts/build_offline_bundle.py --output-dir dist` to include it.
