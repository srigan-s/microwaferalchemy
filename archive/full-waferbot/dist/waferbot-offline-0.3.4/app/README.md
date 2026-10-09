# MicroAlchemy physical robot control (`waferbot`)

For a quick **stationary test of one tape edge**, use
[`scripts/test_line_edge.py`](scripts/test_line_edge.py). It prints the raw and
normalized S1–S4 readings plus `EDGE_OK` when the chosen middle-sensor edge is
stable. It never commands motors and needs no map or navigation configuration.
See [copy, install, and run instructions](docs/LINE_EDGE_TEST.md).

```bash
python3 scripts/test_line_edge.py --edge black-left       # Pi, with smbus2 installed
python3 scripts/test_line_edge.py --demo --samples 12     # Mac, synthetic readings
```

Hardware-independent navigation for autonomous wafer transport, plus a strict
physical adapter for the **Yahboom Raspbot V2** (Raspberry Pi 5, four-wheel
mecanum chassis, four-channel IR line sensor, I2C controller at `0x2B` on bus 1).

The same planner, map, follower, and switcher run against a mock or against real
hardware; only the transport changes.

## Safety first

Read this before touching a floor.

* **Mock is the default.** Real motion needs `--physical` *and* typing `yes` at
  the prompt. There is no flag that skips the prompt.
* **Nothing moves at boot or on import.** No autostart service is installed, and
  constructing the robot performs no I2C write.
* **Arming is explicit.** Motion is refused until a session is armed, and it is
  refused while a fault or emergency stop is latched.
* **The software cannot cut motor power.** A killed process, a wedged I2C bus, or
  a power glitch can leave the controller board holding its last command. Keep a
  physical switch or battery disconnect within reach and use it.
* **Stops win.** Permission checks, the deadline refresh, and the wheel write all
  happen inside one transaction that also holds the cross-process bus lock, so a
  stop or watchdog that latches mid-command can never be followed by a nonzero
  write. `waferbot tick` never clears a live deadline.
* **Wheels off the ground first.** Every first-time check below assumes the
  chassis is supported.
* **Measure before you drive.** The bundled map is illustrative. Physical
  execution refuses a map that is not marked `physical_validated`, refuses
  `positive`/`negative` sides that are not mapped to a real sensor edge, and
  refuses diagonal switching until real geometry is measured. No PWM count is
  ever converted to metres per second without a measured factor.
* **Verify the chassis assumptions.** Floor operation (`follow`, `switch`,
  `execute`, crossing sweeps) refuses to run while the wheel order/sign mapping
  or the sensor channel order/polarity is unverified in configuration. Run
  `waferbot calibrate wheels` and `waferbot calibrate sensors`, then set
  `"verified": true` under `motor` and `sensor` — or pass
  `--ack-verified-config` to acknowledge the assumptions yourself.
* **Physical localization is never invented.** With `--physical` the executor
  accepts only operator confirmation (`--localizer manual`, prompted with the
  wheels stopped) or a real marker reader (`--localizer marker
  --marker-reader module:attribute`). The scripted mock localizer is refused.
* **Bounded, refreshed manoeuvres.** Timed TURN/DOCK moves and the follower's
  recovery re-issue their command faster than the watchdog timeout, respect the
  remaining route budget, and always stop in `finally`. Arrival still requires
  localization evidence.

## Layout

```
src/waferbot/            hardware HAL, sensing, navigation, CLI
  hardware/              I2C transport, motor driver, line sensors, ultrasonic, mock
  sensing/               edge detection, EdgeFollower, edge-switch FSM
  nav/                   track map, Dijkstra/A*, route executor
  calibration/           wheel, sensor, crossing-angle, and speed calibration
  robot.py safety.py     safety controller, watchdog, arming, faults
maps/example_track.json  illustrative graph (NOT measured)
config/                  example robot + navigation configuration
deployment/              install/verify/deploy/offline scripts
scripts/                 stationary sensor test, guarded follower test, bundle generator
tests/                   hardware-free unit and integration tests
HARDWARE_IMPLEMENTATION.md  protocol evidence, wheel mapping, polarity, limits
docs/agent-work/physical-navigation/  phase reports
vendor/yahboom/          read-only vendored Yahboom sources (never imported)
```

## Flashing the SD card

This is a manual step; nothing in this repository writes to a storage device.

1. Install [Raspberry Pi Imager](https://www.raspberrypi.com/software/) on your
   computer.
2. Choose **Raspberry Pi OS (64-bit)** for Raspberry Pi 5.
3. In Imager's **OS customisation** screen set, at minimum:
   * a username and password (the official documentation strongly recommends
     configuring this before flashing),
   * a hostname — `waferbot` makes the default `waferbot.local` work,
   * **Enable SSH** (with password or public-key authentication),
   * your Wi-Fi country/credentials if you are not using Ethernet.
4. Write the image to the new 64 GB card and insert it into the Pi, then boot.
5. Connect from your computer: `ssh pi@waferbot.local` (or the username you set).

References: Raspberry Pi documentation, *Getting started* →
<https://www.raspberrypi.com/documentation/computers/getting-started.html> and
*Configuration* →
<https://www.raspberrypi.com/documentation/computers/configuration.html>.
Those pages returned HTTP 403 to automated fetches from this development
machine, so the wording above was checked against the official documentation
repository (`raspberrypi/documentation`, commit `34dfb87`,
`computers/getting-started/install.adoc` and
`computers/configuration/interfaces.adoc`).

## Install on the Pi

Prerequisites (not bundled, they come from apt):

```bash
sudo apt-get install -y python3-venv python3-dev i2c-tools libi2c-dev
```

Enable I2C (this turns the interface on *and* loads the kernel module at boot):

```bash
sudo raspi-config          # 3 Interface Options -> I5 I2C -> Yes
# non-interactive equivalent:
sudo raspi-config nonint do_i2c 0
sudo reboot                # /dev/i2c-1 appears after a reboot
```

From your MacBook, deploy over SSH (defaults: host `waferbot.local`, user `pi`):

```bash
bash deployment/deploy.sh --build                 # build the bundle, copy it, install
bash deployment/deploy.sh --host waferbot.local --user pi --dry-run
```

Offline alternative (no PyPI access on the Pi):

```bash
# on the connected machine
python3 scripts/build_offline_bundle.py --output-dir dist
scp -r dist/waferbot-offline-0.3.4 pi@waferbot.local:~/waferbot-bundle

# on the Pi
cd ~/waferbot-bundle
bash deployment/offline_install.sh --bundle . --venv ~/.venvs/waferbot   # --no-index only
bash deployment/verify_hardware.sh --venv ~/.venvs/waferbot
```

The bundle ships the deployment scripts at its root, so those two commands work
from the bundle directory. `offline_install.sh` never upgrades pip and never
contacts an index: every wheel comes from `wheels/` and every file is
hash-checked against `manifest.json` first.

On the Pi the CLI lives at `~/.venvs/waferbot/bin/waferbot`; the examples below
use `waferbot` after activating the environment:

```bash
. ~/.venvs/waferbot/bin/activate     # or always use the full path
```

Every physical command below also takes `--config` and `--nav-config`; point
them at the calibrated files you edit (`~/.config/waferbot/robot.json` and
`~/.config/waferbot/nav.json` by default) rather than the unverified templates.

## First hardware session

Use [ROBOT_TESTING.md](docs/ROBOT_TESTING.md) for the complete 0.3.4 copy/install,
calibration and conservative three-second test. It uses `robot.first-run.json`
(maximum 5 PWM counts per wheel) and `nav.first-run.json` (base 4, recovery disabled).
For winding-path diagnosis after the straight test passes, use the documented
`nav.windy-first-run.json` profile (base 5, adaptive curve slowdown, recovery
disabled).
For a rightward tape-crossing experiment with configurable PWM and IR read
frequency, see `waferbot ir-strafe` in [ROBOT_TESTING.md](docs/ROBOT_TESTING.md).
For a constant-angle Black-Left to Black-Right crossing sweep, the same guide
documents `waferbot edge-angle-sweep` and its per-read timing CSVs.
The commands below are a reference for an already calibrated installation.

For a separate, **stationary** AprilTag camera/OLED test, see
[APRILTAG_TESTING.md](docs/APRILTAG_TESTING.md). `waferbot tags` recognizes
`tag36h11`, prints/logs the ID, briefly displays it on the stock Yahboom OLED,
and restores its normal screen. It does not access motor control.
`waferbot oled-test --duration 3` checks the packaged OLED driver without
opening the camera or motors.

Run these in order. Each physical command prompts before it moves anything.

**1. Sensors (read-only, no motion)**

```bash
waferbot sensors --physical --samples 5 --config ~/.config/waferbot/robot.json
waferbot calibrate sensors --physical --config ~/.config/waferbot/robot.json
```

Expected raw values with the vendored firmware: `0` = black line detected,
`1` = white. The example mapping is S1→bit 2, S2→bit 3, S3→bit 1, S4→bit 0
(see `HARDWARE_IMPLEMENTATION.md` section 7). Confirm both polarity and order
here; adjust `config/robot.example.json` (`sensor.bit_for_channel`,
`sensor.black_is_raw_zero`) if your board differs, then set `"verified": true`
under both `motor` and `sensor` once the calibration runs pass.

**2. Motor test (wheels off the ground)**

```bash
waferbot motor-test --physical --speed 5 --config ~/.config/waferbot/robot.json
```

Type `lifted` when asked. Each wheel id is driven on its own and you are asked
whether it turned forward. Until you answer `yes` for all four, the report says
`uncertain_until_confirmed`; wheels reported backwards are listed under
`suggested_invert`, which you set with `motor.invert` in the robot config.
Standing assumption (unverified by vendored source): id 0 front-left,
1 rear-left, 2 front-right, 3 rear-right.

**3. Follow one edge on a straight section**

```bash
waferbot follow --physical --edge black-left --duration 3 \
  --config ~/.config/waferbot/robot.json --nav-config ~/.config/waferbot/nav.json
```

Starts at PWM counts from `follow.base_speed`, corrects with the PD gains, slows
at junctions, recovers from a bounded loss of the line, and always stops the
wheels when it returns. `--iterations N` bounds the loop for bench tests.

**4. Switch to the opposite edge**

```bash
waferbot switch --physical --config ~/.config/waferbot/robot.json \
  --nav-config ~/.config/waferbot/nav.json \
  --from-edge black-left --to-edge black-right \
  --location B2 --route-id manual --authorize
```

Without `--authorize` (with `--location` and `--route-id`) the command refuses to
move: the authorization binds the source node, both edges, and the switch
location. The manoeuvre only starts after the current edge is established, only
resumes after several consecutive fresh readings of the opposite edge, and only
finishes after the destination is confirmed (a stopped operator prompt on
hardware). Lateral strafing is the default; a diagonal crossing needs measured
geometry *and* a measured `max_travel_m` travel budget.

**5. Plan and execute A+ → B- → D-**

```bash
waferbot map --map maps/example_track.json
waferbot plan --start A+ --goal D- --via B-
waferbot plan --start A+ --goal D- --via B- --algorithm astar
waferbot execute --start A+ --goal D- --via B- --dry-run        # plan only
waferbot execute --physical --start A+ --goal D- --via B- \
  --config ~/.config/waferbot/robot.json \
  --nav-config ~/.config/waferbot/nav.json \
  --map ~/.config/waferbot/track.json --localizer manual
```

`--dry-run` never constructs a transport, never opens a bus, and never writes
stop blocks. Physical execution requires a
`physical_validated` map with both edge sides mapped, and it requires a measured
`nav.counts_to_mps` factor so each edge's `speed_limit_mps` can actually cap the
motors; the bundled example map is refused on purpose and an uncalibrated speed
is refused with instructions. `--localizer manual` prompts only with the wheels
stopped; `--localizer marker` needs a real reader via
`--marker-reader package.module:factory`. Elapsed time and edge polarity are
never treated as arrival.

Optional obstacle hook (off by default): add `--obstacle-distance-mm 200` to a
physical motion command to start the ultrasonic monitor; a reading below the
threshold triggers `OBSTACLE_DETECTED` and an emergency stop, and a ranging read
failure raises `SENSOR_FAILURE` instead of driving on.

**6. Stop**

```bash
waferbot stop --physical      # latch a stop AND write real stop blocks over I2C
waferbot stop --status        # who owns the bus, is a stop latched
waferbot stop --clear         # release the latch after you have checked the robot
```

The latch is checked before every motor command; `stop --physical` also takes
the cross-process bus lock and writes the four stop blocks itself, so an absent
or dead owner still leaves the controller stopped. It reports
`stop_bytes_written`, `bus_lock_acquired`, and any error instead of claiming the
lock proves the motors stopped. `stop --clear` is refused while a motion session
still owns the bus, because that session could resume and overwrite the stop.

## Mock mode (no hardware required)

Every command defaults to an in-memory mock, so the whole stack is testable on a
laptop:

```bash
waferbot sensors --samples 3
waferbot map
waferbot plan --start A+ --goal D- --via B-
waferbot execute --start A+ --goal D- --via B-          # completes in milliseconds
waferbot follow --edge black-right --iterations 20
waferbot switch --from-edge black-left --to-edge black-right --location B2 \
  --route-id demo --authorize
waferbot motor-test --mock
```

## Calibration

**Geometry (needed for diagonal switching and for trusting map distances).**
Measure on the real track and fill `nav.geometry`:

| Field | Meaning |
| --- | --- |
| `tape_width_m` | width of the black tape |
| `sensor_spacing_m` | centre-to-centre distance between adjacent sensors |
| `sensor_detection_width_m` | width each sensor actually detects |
| `available_crossing_distance_m` | straight distance available along the tape for the manoeuvre |
| `safety_margin_m` | margin you want either side of the tape |
| `robot_width_m` | chassis width |
| `switching_speed_mps` | measured speed used while crossing |
| `sampling_rate_hz` | sensor sampling rate during the crossing |
| `lateral_clearance_m` | measured free space to the side of the track |
| `required_confirmations` | how many fresh confirmations the controller needs |

Then set `"measured": true`. The minimum angle is
`atan((tape_width + 2 * safety_margin) / available_crossing_distance)`; the
evaluation also reports the sensor footprint, the lateral sampling distance, the
observability margin (lateral speed, real polling rate, detection width, and the
required confirmations), forward clearance, and lateral robot clearance, and
refuses an angle that fails any of them. These checks are necessary rather than
sufficient: they bound the geometry, they do not prove an unambiguous real
transition.

Physical switches additionally need `switch.max_travel_m`, a measured
conservative straight-line budget in metres. It is converted to a time budget
using the commanded counts and `counts_to_mps`, shared across the crossing,
search, and resume phases, and it is never treated as arrival evidence.

**Crossing sweep.**

```bash
waferbot calibrate crossing --physical --angles 12,15,20,25 --attempts 3 \
  --csv crossing_sweep.csv
```

Each attempt is logged with success/failure, duration, final edge, and reason.
The CSV keeps every raw attempt plus a per-angle summary row, and no best angle
is reported when every attempt failed. On hardware each attempt stops the robot,
asks you to confirm the source repositioning, authorizes that attempt, and only
then crosses.

**Speed (only if you want `speed_limit_mps` to mean something).**

```bash
waferbot calibrate speed --instructions
waferbot calibrate speed --counts 50 --distance-m 1.0 --duration-s 8.5
```

Put the reported `recommended_counts_to_mps` into `nav.counts_to_mps`. It is the
**fastest** measured m/s-per-count inflated by the conservatism factor, so the
derived PWM caps stay below the requested limit; re-measure for strafing,
different payloads, and different battery charge. Until you calibrate, telemetry
keeps `estimated_speed_mps` empty and reports PWM counts.

## Telemetry

```bash
waferbot execute --start A+ --goal D- --log-csv run.csv
```

One row per event (sensor read, motor command, controller decision,
localization, fault) with timestamp, raw and normalised sensor values, detected
and target edge, current and target node, per-wheel commands, estimated speed in
counts (and in m/s only when calibrated), fault code, and free-form notes.

## Configuration

* `config/robot.example.json` – controller address/bus, speed bound, wheel
  labels, per-wheel polarity inversion, sensor register/polarity/mapping,
  `verified` flags for the wheel and sensor mappings, watchdog timings.
* `config/nav.example.json` – follower gains and rate, switch mode and bounds,
  measured geometry, execution bounds, telemetry path, map path, optional speed
  calibration.
* `maps/example_track.json` – directed adjacency list with node and edge fields,
  including `edge_side` (positive/negative) and `direction` (forward/reverse) as
  separate concepts.

CLI exit codes: `0` success, `2` usage/argument problem, `3` refused, faulted, or
blocked by an uncalibrated map/geometry.

## Development

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -e ".[test]"
python -m pytest            # no hardware required
```

`pytest` never opens a bus: the mock transport records every transfer and can
inject write/read faults, so protocol, fault, watchdog, and route behaviour are
all covered without a Pi. The physical-mode tests inject a transport and a
prompt, so the hardware code paths (manual localization, stop-byte writing,
obstacle monitor, verified-config gate) are exercised without a robot, and the
deployment tests run the generated remote commands through ssh/rsync shims in a
temporary fake Pi layout.
