# Physical navigation 0.2.0 final report

## Result

The repository now contains a hardware-independent navigation stack and a
strict Yahboom Raspbot V2 adapter. It includes the four-channel oriented-boundary
follower, adaptive curve speed, bounded edge switching, crossing geometry,
editable JSON graph, Dijkstra and A*, localization-driven route execution,
typed faults, watchdog/process stop controls, calibration commands, CSV
telemetry, CLI, Raspberry Pi deployment scripts, and an offline release.

The original repository contained vendored Yahboom demonstrations but no
MicroAlchemy application package, ROS/Gazebo integration, or existing graph
navigator to extend. The vendor tree remains unchanged and is used only as
protocol evidence. ROS is not required.

## Hardware interface

Existing vendor evidence establishes I2C bus 1, address `0x2B`, motor register
`0x01`, line register `0x0A`, motor ids 0..3, direction byte and mecanum mixing.
The application implements those transfers through `smbus2` behind an injected
transport. Default physical line mapping S1..S4 = bits 2,3,1,0 and raw zero =
black remain operator-verified settings. L1/L2 front/rear labels and individual
wheel inversion must be confirmed on the actual chassis.

No independent controller-board watchdog was verified. The software watchdog,
shared stop latch and serialized stop writes reduce risk but cannot stop a
controller holding its last command after process death or a wedged I2C bus. A
physical power cutoff remains required.

## Follower evidence

The controller follows a selected oriented black/white transition. It performs
stationary acquisition, rejects stale/future/replayed readings, uses filtered PD
correction with a slew limit, slows at junctions and as steering demand rises,
and always applies a bounded duration and final stop. The conservative and
windy-path profiles cap wheels at 5 PWM through `robot.first-run.json`; recovery
is disabled so line loss stops rather than searches.

The deterministic tape model derives sensor bits from commanded wheel motion,
chassis pose, tape width and centerline. Tests cover both boundary orientations,
offset/heading errors, straight and curved tape, alternating tighter turns,
noise/dropouts, missing tape, sensor faults and wrong-sign/no-feedback controls.
The older fixed-speed low-authority profile loses the synthetic tight curve that
the windy profile tracks. This is synthetic evidence, not physical tuning.

## Requirements mapping

- HAL/protocol/sensors: `src/waferbot/hardware/`, `robot.py`, `kinematics.py`.
- Edge detection/follow/switch: `src/waferbot/sensing/`.
- Geometry/config: `geometry.py`, `navconfig.py`, `config/`.
- Graph/planners/executor: `src/waferbot/nav/`, `maps/example_track.json`.
- Localization/safety/watchdog/process stop: `localization.py`, `safety.py`,
  `watchdog.py`, `process.py`.
- Calibration/telemetry/CLI: `src/waferbot/calibration/`, `telemetry.py`, `cli.py`.
- Operator tools: `scripts/test_line_edge.py`, `scripts/test_line_follow.py`.
- Pi deployment/offline release: `deployment/`, `scripts/build_offline_bundle.py`.
- Hardware decisions: `HARDWARE_IMPLEMENTATION.md`.
- Exact operator sequence: `docs/ROBOT_TESTING.md`.
- File inventory: `docs/PHYSICAL_NAVIGATION_FILES.md`; packaged hashes are in
  `dist/waferbot-offline-0.2.0/manifest.json`.

## Verification

The final automated test log is `logs/final-020-tests.log`. Offline installation
and the 0.1.0 to 0.2.0 upgrade are recorded in `logs/final-020-offline.log`.
The installed wheel CLI is exercised from outside the repository, including the
mock A+ to B- to D- route. Shell syntax, compilation, bundle hash integrity,
package contents and the unchanged original-file baseline are checked at the
release boundary.

At the initial acceptance boundary, **495 tests and 10 subtests passed** in the
hardware-free suite. The five-count revision is verified in a subsequent run.
The rebuilt manifest contains 68 files with matching SHA-256 hashes and all
required 0.2.0 scripts, configs, guides and wheels. All 76,997 files from the
pre-task baseline remain present and byte-identical.

No Raspberry Pi, motors, sensors, tape, SSH target or SD card was operated during
implementation. Physical results remain unverified until the operator follows
`docs/ROBOT_TESTING.md`. The bundled graph is illustrative and deliberately
refused for physical execution; a measured, validated track and real localization
confirmations are required for autonomous routes.

## Five-count winding-path revision

The first-run robot profile now caps each wheel at 5 PWM counts. Straight and
winding follow profiles use base speeds 4 and 5, respectively, with PD gains
2.0/0.06 and 2.5/0.08. A deterministic tight alternating-curve regression checks
both edge orientations and confirms every motor command stays within the cap.
At such low duty the physical motors may stall; only a raised-wheel test can
establish whether five counts is usable on this chassis. The operator sequence
is in `docs/ROBOT_TESTING.md`.

The full hardware-free suite now passes **501 tests and 10 subtests**; its log is
`logs/final-five-tests.log`. The rebuilt 0.2.0 offline bundle passes its 68-file
hash check and an offline installation from outside the repository; the installed
CLI reports version 0.2.0 and plans the mock route. That evidence is in
`logs/final-five-offline.log`. The generated 0.1.0 bundle, stale source ZIP and
Python caches were moved out of the repository; the 0.2.0 bundle remains the
release to copy to the Pi. All 76,997 original files still hash-match their
baseline. No physical test was run.

## Version 0.3.0 stationary AprilTag diagnostic

The repository has no previous AprilTag implementation. The stock Yahboom
demos provide the OpenCV USB camera pattern and reference an external
`Yahboom_OLED` driver. The new `waferbot tags` command recognizes `tag36h11`,
logs the ID and image corners, shows a brief OLED notice, then restores its
configured normal screen. It does not create a motor session. Tag IDs are
available as string `marker_id` values for a future measured node map; this
diagnostic does not claim pose or arrival at any graph node.

The full suite passes **509 tests and 10 subtests**, including a real OpenCV
decode of a generated tag and mocked camera/display cleanup. The 0.3.0 base
bundle passes its 70-file manifest check and no-network install smoke test;
OpenCV remains an optional Pi camera dependency installed separately. Logs are
`logs/apriltag-full-tests.log` and `logs/apriltag-offline-install.log`.
The stock Pi OLED driver and physical camera have not been tested here.

## Version 0.3.1 packaged OLED correction

The user confirmed that AprilTag detection works on the real Pi with
`--display none` (26 detections), that the external `yahboom_oled.py` path is
absent, and that `i2cdetect -y 1` shows the OLED at `0x3c`. The referenced
Yahboom key-reading notebook documents the `Yahboom_OLED` API but does not
contain its driver. The new packaged `waferbot.vision.yahboom_oled.Yahboom_OLED`
implements those methods with the existing `smbus2` dependency and a 128×32
SSD1306 framebuffer. `waferbot oled-test --duration 3` writes `Waferbot` /
`Ready` without opening camera or motor control. `--display none` continues to
skip OLED initialization; an explicit `--oled-driver` override remains.

The full suite passes **512 tests and 10 subtests**. Unit tests verify OLED
transfers address `0x3c`, both display rows contain pixels, the OLED-only CLI
avoids camera and motor sessions, and the display-free tag path avoids the
OLED. The rebuilt 0.3.1 offline bundle installs with no network use. Evidence
is in `logs/oled-final-tests.log` and `logs/oled-offline-install.log`. Physical
OLED rendering is awaiting the operator's three-second Pi test.
