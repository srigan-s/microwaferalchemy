# Astra acceptance review — changes required

The implementation is substantial and the existing mock suite passes, but the following concrete safety and deployment issues prevent acceptance. Apply this as one consolidated correction bundle, adding regression tests for the observed failures. Root retains this review file. Preserve baseline files and rebuild the distribution after fixes.

## 1. Stop/motion atomicity and watchdog

`robot.py:drive_wheels` checks arming and external-stop state outside the lock used for motor writes. A stop/watchdog can latch and write zeros between the checks and the subsequent command; the command then writes nonzero values after the stop. Root reproduced this by injecting emergency_stop immediately before the original MotorDriver.set_speeds: the final four payloads were nonzero while e-stop remained latched. Serialize the complete permission/deadline/write transaction with safety transitions and check external stop under the cross-process transaction guard. Avoid deadlocking nested flock acquisitions. Add deterministic barriers testing both watchdog/e-stop and cross-process stop races. Preserve primary faults/e-stop state across later faults.

`Robot.tick()` unconditionally cancels the independent watchdog even when nothing expired. Root reproduced deadline becoming None after an ordinary early tick. Fix it; synchronous expiry must also latch MOTION_TIMEOUT. Verify watchdog refresh/cancel/expiry races do not cancel a newly valid command.

`waferbot stop` currently only creates a file and takes/releases a lock, never sends stop bytes. Ensure a live physical owner acknowledges a stop promptly independently of its controller loop, and provide an explicit physical zero-command path for an absent/dead owner. Do not claim bus lock acquisition proves motors stopped. Refuse clearing the stop latch while a live motion session can resume. Handle bus-lock TimeoutError as a latched communication fault. Close the actual transport during lifecycle cleanup.

## 2. Physical localization must never be fabricated

`cli.py` parser defaults execute `--localizer scripted`, and `_arrival_monitor_for` returns MockArrivalMonitor even with --physical. Root confirmed this directly. Physical mode MUST select manual or real marker localization and reject mock/scripted localization before motion.

`cmd_switch` and `_calibrate_crossing` use `_fixed_localizer`, `_FixedMonitor`, and `confirm=lambda: True` in physical mode. Replace these with stopped operator confirmations (or actual marker evidence); --location is a requested identifier, not observed localization. Every physical sweep attempt must stop and explicitly confirm repositioning to the source, authorize that attempt, confirm destination, and record failure without fake success. No mock helper can be reached by physical execution. Test these paths with an injected physical transport and prompt, not hardware.

## 3. Fresh evidence and truthful arrival

`EdgeDetector.update` uses poll time instead of the observation timestamp for consecutive freshness. Reusing one LineReading three times currently becomes stable ([False, False, True] reproduced). Reject nonfinite/future/nonincreasing readings; reset stability on duplicates, backwards time and gaps. Validate raw/normalized channel values; fix `edge_byte_for` to respect configured inverted polarity.

`follower._check_arrival`, `executor._confirm_node`, and `switching._confirm_destination` accept arbitrary matching objects without timestamp/confidence validation. `Localization.matches` even accepts a matching marker_id when node_id positively names another node. Centralize freshness, finite confidence, identity and marker-to-node mapping checks; validate at every start/arrival/switch confirmation. Repeated stale marker frames must not count as multiple samples. Preserve actual localization evidence rather than synthesizing source='arrival' with confidence=1. Tests: stale/future/low-confidence/wrong-node/duplicate evidence never authorizes motion or arrival.

## 4. Motion loops, bounds and controller behavior

WheelCalibrator defaults to drive/sleep(1)/stop against a 0.5-second physical watchdog, so the first wheel times out and later probes cannot run. TURN/DOCK and follower recovery have the same one-shot-command/long-sleep problem. Use a common bounded, interruptible command-refresh loop (and finally-stop) at a rate faster than the watchdog. Exercise it with the watchdog enabled, not exclusively watchdog=False mocks. Respect the caller's remaining route/controller budget in every phase and recovery subphase.

Follower only declares line loss when all four sensors are white. It treats persistent BOTH_BLACK and the wrong edge as ordinary tracking indefinitely, and ignores stale/debounce status for motion. Allow brief ambiguous junction observations with slowdown, but bound ambiguity/wrong-edge duration and stop/recover without declaring a valid transition. Recovery needs fresh stable target observations, periodic command refresh and the global deadline. Reject NaN/infinite/negative duration and invalid iteration/count arguments before motor writes; current follow(duration=NaN) bypasses its deadline forever. Enforce integer PWM/count fields in navconfig rather than accepting arbitrary floats.

## 5. Switch geometry, travel and graph commands

SwitchConfig.max_travel_s is unused. There is no configurable crossing distance enforcement. Enforce one shared motion budget across crossing, search and resume, plus a measured conservative speed-based distance budget in metres for physical switches (explicitly estimated without odometry). Never treat the estimate as arrival. Pass counts-to-speed calibration into switchers and require it for physical distance bounds. Check every moving phase before/after commands.

Executor ignores MapEdge.crossing_angle_deg and instead always uses global switch configuration. Make each route's switch action executable with its declared geometry/mode, including an explicit lateral representation (e.g. 90 degrees) and diagonal angles. Diagonal blending currently exceeds the calibrated wheel cap (forward+lateral > scalar speed); scale safely without changing its angle. Clamp no sub-one-count speed ceiling up to 1: refuse an unrepresentable ceiling. Apply calibrated speed caps to TURN and DOCK as well as FOLLOW/SWITCH.

TURN always rotates right and ignores headings; direction=reverse is also ignored throughout. Define/document the heading and travel-direction conventions, execute the signed heading change and reverse travel where supported, and refuse unsupported combinations rather than silently moving a different way. TURN/DOCK may remain timed calibration maneuvers without odometry if they have hard shared budgets, speed caps, watchdog refresh and independent arrival/pose confirmation; they must not pretend to measure distance.

Geometry currently compares robot width to the available *longitudinal* crossing distance as though it were lateral free space, and permits large between-sample jumps using pitch+footprint. Keep the user's atan bound, add explicit measured lateral clearance, and make sensor observability checks conservative using lateral speed, actual sampling rate, detection footprint and required confirmation counts. Do not claim all checks guarantee an unambiguous real transition. Validate physical geometry against actual commanded rate/speed, not unrelated config numbers.

## 6. Route/map and planner correctness

`RouteExecutor._validate_steps` does not validate route start/end, node sequence, step continuity, membership in the current map, enabled status, or mandatory waypoints. Validate the actual route before motion, including forged/stale routes and disabled connections.

STOP currently advances to a different destination with no localization. Restrict it to a physically stationary self/coincident action with explicit positive localization if the identifier changes. Planner skips STOP in its metric-consistency test, allowing a zero-cost teleport to break A* admissibility. Reject spatial teleport STOP maps and check all enabled edges in heuristic validity. Remove double-counting g in search priority (`candidate + _priority(...candidate...)` currently yields 2g+h).

Map validation must enforce source/destination edge-side compatibility and direction conventions so a FOLLOW step cannot silently start on a different side than the previous node. Use real adjacency lists internally. Keep the example clearly fictitious and physical-gated.

## 7. Cleanup and reported success

Switcher/executor build frozen completed=True results inside try, then finally catches stop failures by setting a local variable. The already-constructed returned result remains successful and omits the cleanup fault. Follower also preserves ARRIVAL despite a cleanup fault. Return success only after mandatory shutdown succeeded; test a last-stop failure after otherwise successful follow/switch/route. Preserve first failure and append stop failures, never mislabel a sensor failure as switch timeout. Every prompt and any localization provider that requires stop must see stopped wheels.

## 8. Calibration correctness

`compute_counts_to_mps` uses min(measured_ratio)*0.8, while caps divide by this factor: this raises allowed PWM and can exceed the actual speed limit. Use a conservative upper bound on m/s-per-count (e.g. max(ratios)/0.8), finite positive validated samples, document uncertainty and remeasure for strafing/load/battery. Test that commands never exceed the requested speed under the fastest measured ratio. Fix instructions (currently mock follow command and contradictory fastest/min wording).

Sensor calibration assumes zero is black and picks any zero bit even from ambiguous/noisy frames. Measure all-white baseline and isolated-channel changes, infer either polarity, require consistent one-bit changes across samples and reject mixed/ambiguous cases. Wheel report confirmed must require all four mappings plus requested direction checks; partial-wheel testing is not global verification. Keep raw results and observed final edge in sweep CSV, include attempt success-rate summaries, and do not pick a best angle when every attempt failed.

## 9. Deployment and actual offline packaging

Bundle layout has scripts under `app/deployment/`, but deploy.sh/README.txt/runbook invoke nonexistent top-level deployment/*.sh. Deploy also cd's into REMOTE_DIR then passes the same relative REMOTE_DIR as --bundle, doubling it. Its quoted default '$HOME/.venvs/waferbot' never expands on the Pi, and custom --venv is interpolated into remote shell without safe quoting. Correct all paths and remote argument handling; default remote HOME expansion must be deliberate and safe. Support stock macOS rsync (do not assume --info=progress2). Add command-shim integration tests that actually execute generated remote shell commands in a temporary fake Pi layout, proving path and argument correctness without SSH or apt.

offline_install.sh runs pip install --upgrade pip online before its --no-index step. Remove all network requests from the offline path (also install_pi --bundle where it claims no network for Python packages). Test with network access disabled/invalid index and inspect every pip invocation. Verify the real bundle's manifest and required wheels, including README.txt (currently written after manifest creation), retain dependency license metadata and pin recorded dependency versions for reproducibility. --no-download currently deletes the existing wheel cache instead of reusing it; make it truthful or remove it. Rebuild final bundle and no-index install from its shipped script location.

Install must configure/check current user's I2C group membership, explain reconnect/reboot, and make logging/config usable. `verify_hardware --bus/--address` currently changes only i2cdetect, not the sensor CLI; propagate config. Its 'no motor commands' claim is false because sensors Session.close stops all motors. Implement a truly read-only diagnostic lifecycle, while fault cleanup for a motion session still stops. Avoid opening a bus at all for --physical --dry-run; current dry run opens and closes hardware, emitting stop writes. Tests must assert zero hardware constructions/writes for dry run.

CLI currently loads maps/example_track.json even for sensors/motor tests, so installed-wheel commands from any other working directory fail. Make hardware-independent commands not require a map, and ship a packaged default example resource or resolve explicitly installed resources for map/planning. Close partially-created resources/ownership on session-construction failure. All first-hardware commands in README must actually load the user's calibrated config/nav files (currently edits are instructed but commands keep loading unverified defaults). Include venv activation/PATH setup.

## 10. Obstacle hook, telemetry and complete evidence

Ultrasonic ObstacleMonitor says CLI wires it to emergency_stop but CLI never constructs it. Provide an explicit optional configured obstacle-monitor path for physical sessions, disabled/unverified by default, with SENSOR_FAILURE on read failures when enabled; do not silently swallow sensor failure and drive. Validate documented protocol from vendor, keep unknown ranging limits configurable. Test its actual session integration.

Robot faults/stops are not consistently logged, switching ignores telemetry, and follower logs expected destination as current_node. Log stop commands, fault state and switch phases; preserve actual current_node vs target_node. CSV must not claim a commanded target edge is observed on failed crossing. Protect logger lifecycle/concurrent fault logging.

Refresh docs/hardware assumptions/test counts and provide a complete exact FILES manifest (including test names, package __init__ files and generated bundle contents or linked manifest). Full suite, installed-wheel CLI from an unrelated cwd, physical-mode tests with injected hardware, enabled-watchdog tests, command-shim deployment, truly offline shipped-script installation, and rebuilt baseline/manifest verification are required. No live robot/SSH/apt/disk operations. Report corrected results honestly and list any residual hardware limits.
