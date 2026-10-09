# Test the Yahboom Raspbot V2 on Raspberry Pi OS

The Pi is reachable at `pi@waferbot.local`; no SD-card reflash is needed.
These are operator commands, not a claim that hardware was tested during
implementation. Start with stationary sensors, then raised wheels, then a short
floor run. No service starts motion at boot.

For the independent camera/OLED AprilTag diagnostic, see
[APRILTAG_TESTING.md](APRILTAG_TESTING.md). That command requires no motor test.

## 1. Copy the completed release from the Mac

From the repository on the Mac:

```bash
rsync -az --partial dist/waferbot-offline-0.3.6/ pi@waferbot.local:waferbot-test/
ssh pi@waferbot.local
```

The release is already built. To rebuild after source edits, use a development
virtual environment with `pip`, `setuptools>=68`, and `wheel` installed, then
`python scripts/build_offline_bundle.py --output-dir dist`. The ordinary build
can download dependencies; `--no-download` reuses wheels in the target bundle.

## 2. Install on the Pi

Run in the SSH terminal, as `pi` (do not run waferbot itself with sudo):

```bash
sudo apt-get update
sudo apt-get install -y python3-venv i2c-tools
sudo raspi-config nonint do_i2c 0
bash ~/waferbot-test/deployment/offline_install.sh --bundle ~/waferbot-test
source ~/.venvs/waferbot/bin/activate
waferbot --version
```

Expect `waferbot 0.3.6`. The Python install verifies the manifest, installs with
`--no-index`, and upgrades an older waferbot installation from bundled wheels.
The apt packages above must be installed before going offline.

If `/dev/i2c-1` is absent, reboot and reconnect. If access is denied, run
`sudo usermod -aG i2c "$(id -un)"`, then log out and reconnect. Activate the venv
again after reconnecting.

Create a separate first-run config. If you already calibrated it, preserve the
mapping and inversion while enforcing the new five-count wheel cap:

```bash
mkdir -p ~/.config/waferbot ~/waferbot-logs
python - <<'PY'
from pathlib import Path
import json
import shutil
source = Path.home() / 'waferbot-test/app/config/robot.first-run.json'
target = Path.home() / '.config/waferbot/robot.first-run.json'
if target.exists():
    data = json.loads(target.read_text())
    data['motor']['max_speed'] = 5
    target.write_text(json.dumps(data, indent=2) + '\n')
    print('Preserved calibration and capped motor.max_speed at 5:', target)
else:
    shutil.copyfile(source, target)
    print('Created unverified first-run configuration:', target)
PY
```

## 3. Stationary sensor test (no motor commands)

```bash
python ~/waferbot-test/app/scripts/test_line_edge.py --edge black-left
```

Keep the robot stationary and put the boundary between S2 and S3. Viewed from
behind the robot, S1..S4 run left to right. Normalized `1` means black and `0`
means white. For BLACK_LEFT, S2 black/S3 white becomes `EDGE_OK` after three
matching samples (`1100` and `0100` are centered examples). For BLACK_RIGHT,
use `--edge black-right`; S2 white/S3 black is the target (`0011` or `0010`).
Both-black and both-white are ambiguous. Ctrl+C exits.

Move black tape under each channel individually. The corresponding normalized
column must change to 1. Vendor evidence suggests raw zero means black and the
bits are S1..S4 = `2,3,1,0`; verify this on your board. The standalone script's
`--bits` and `--black-value` options are independent of the robot JSON.

For config-aware reads and calibration:

```bash
waferbot sensors --physical --samples 20 \
  --config ~/.config/waferbot/robot.first-run.json
waferbot calibrate sensors --physical \
  --config ~/.config/waferbot/robot.first-run.json
```

The calibration asks for white and individual black-channel samples. It prints
suggested `sensor.bit_for_channel` and `sensor.black_is_raw_zero`; it does not
edit your configuration. Optional communication check after installation:

```bash
bash ~/waferbot-test/deployment/verify_hardware.sh \
  --venv ~/.venvs/waferbot --config ~/.config/waferbot/robot.first-run.json
```

### IR polling experiment: strafe right across the tape

Use this after wheel direction and sensor mapping are verified. Viewed from
behind the robot, place the **whole sensor bar to the left of the tape** so the
stationary baseline row reads white on all four channels. Leave clear floor to
the right, keep the power switch within reach, and use a short duration. This
is open-loop motion: it does not steer from the tape or stop automatically at
the far edge.

```bash
mkdir -p ~/waferbot-logs
~/.venvs/waferbot/bin/waferbot ir-strafe --physical \
  --speed-pwm 5 --sample-rate-hz 100 --duration 2 \
  --config ~/.config/waferbot/robot.first-run.json \
  --output-csv ~/waferbot-logs/ir-strafe-5pwm-100hz.csv
```

Type `yes` at the prompt. `--speed-pwm` is an integer bounded by the robot
config's five-count wheel limit; `--sample-rate-hz` requests 1–500 reads per
second; `--duration` is limited to ten seconds. Start with two seconds and
reduce it if the cart could run out of clear floor. To compare conditions,
repeat with different `--speed-pwm` and `--sample-rate-hz` values and a new CSV
name each time. The summary prints the **achieved** host read rate, read count,
pattern transitions, and missed read deadlines. Each CSV row gives the raw
byte, raw bits, normalized black bits, monotonic read timestamps, interval, and
read latency. The first row is stationary (`phase=baseline`); the others are
collected while the right-strafe command is active. A 100 Hz target means a
10 ms requested interval. It does not establish the IR board's internal update
rate; the tape crossing gives only a small number of value transitions. An
external timed optical stimulus would be needed to measure that separately.
Ctrl+C or `waferbot stop --physical` from another SSH terminal stops the run.

### Constant-angle edge-switch characterization

`edge-angle-sweep` is an open-loop measurement of the sensor-defined transition
from `BLACK_LEFT` to `BLACK_RIGHT`. It uses one fixed four-wheel command per
trial, so there is no steering correction or search. Angle is measured from
forward travel along the tape: 0° is forward, 90° is leftward across the tape.
Place the Black-Left boundary between S2 and S3 before **each** trial. The
command stops and asks you to reposition while the wheels are stationary.
First try one short crossing with clear space forward and left:

```bash
~/.venvs/waferbot/bin/waferbot edge-angle-sweep --physical \
  --angles 30 --attempts 1 --speed-pwm 5 --sample-rate-hz max \
  --duration 3 --confirm-ms 50 \
  --config ~/.config/waferbot/robot.first-run.json \
  --output-dir ~/waferbot-logs/edge-angle-pilot
```

If the pilot moves in the expected direction and the course has enough clear
floor, run the 10° increment sweep in a **new** output directory:

```bash
~/.venvs/waferbot/bin/waferbot edge-angle-sweep --physical \
  --angles 10:80:10 --attempts 3 --speed-pwm 5 --sample-rate-hz max \
  --duration 3 --confirm-ms 50 \
  --config ~/.config/waferbot/robot.first-run.json \
  --output-dir ~/waferbot-logs/edge-angle-sweep-01
```

`max` requests unthrottled reads; each trace reports the achieved rate. You can
instead request 1–500 Hz. `summary.csv` has one row per attempt, including the
requested and commanded angles, wheel vector, success, and milliseconds from
the last `BLACK_LEFT` reading to the first sustained `BLACK_RIGHT` reading.
Each per-trial CSV includes every raw byte, all four normalized channels,
monotonic timestamps, edge-state changes, and the interval since the previous
change. A target must remain `BLACK_RIGHT` for at least three reads and 50 ms.
The report gives the smallest **tested** angle where all repeats met that
sensor rule and the fastest reliable angle by median switch time. At five PWM
counts, integer wheel commands cannot represent
every 10° increment exactly; the report flags duplicate commands (40° and 50°
currently coincide). Compare the commanded angle and wheel vector, not only
the requested label. Neither this experiment nor the opposite sensor edge
proves arrival at a navigation node or a safe path through obstacles.

## 4. Motors: raise all wheels off the ground

Keep the power switch within reach. Test one wheel for 0.2 seconds:

```bash
waferbot motor-test --physical --wheel 0 --speed 5 --duration 0.2 \
  --no-directions --config ~/.config/waferbot/robot.first-run.json
```

Repeat with ids `1`, `2`, and `3`. Follow the explicit `yes`/`lifted` prompts.
Verify physical wheel position and forward direction. Default labels are id0
front-left, id1 rear-left, id2 front-right, id3 rear-right; only the left/right
pairing is established by vendor source. `motor.invert` corrects polarity.
`wheel_labels` only changes labels: it does not remap motor ports. Resolve any
physical port mismatch before driving. With the wheels still raised, test the
six chassis movement patterns:

```bash
waferbot motor-test --physical --speed 5 --duration 0.2 \
  --config ~/.config/waferbot/robot.first-run.json
```

Edit the first-run configuration after checking the actual wheels and sensors:

```bash
nano ~/.config/waferbot/robot.first-run.json
```

Preserve the JSON structure. Set the measured sensor bit order/polarity and
motor inversion, then set `motor.verified` and `sensor.verified` to `true` only
after those checks pass. Keep `motor.max_speed` at **5**. Floor operation
refuses unverified settings. Do not use an acknowledgement flag to skip these
first checks.

## 5. Conservative first physical line-following test

Put the robot on a straight tape boundary, aligned along the tape, with the
selected boundary between S2 and S3. Clear the run-out and keep the power switch
within reach. The new PID profile defaults to 300 Hz IR polling and 100 Hz
control, with base PWM 4, a five-count wheel limit, stationary acquisition,
Ki=0, and **recovery disabled**. The shipped controller is disabled until this
operator-owned copy is explicitly enabled after stationary and raised-wheel
checks. Verify sensor positions before interpreting millimetre values:

```bash
cp ~/waferbot-test/app/config/nav.first-run.json ~/.config/waferbot/nav.pid-first-run.json
python - <<'PY'
from pathlib import Path
import json
path = Path.home() / '.config/waferbot/nav.pid-first-run.json'
data = json.loads(path.read_text())
data['follow']['controller_enabled'] = True
path.write_text(json.dumps(data, indent=2) + '\n')
print(path)
PY
```

Then run this three-second test with the stop command ready in another SSH
terminal:

```bash
waferbot follow --physical --edge black-left --duration 3 \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/.config/waferbot/nav.pid-first-run.json \
  --control-log-csv ~/waferbot-logs/first-follow-control.csv \
  --log-csv ~/waferbot-logs/first-follow-telemetry.csv
```

Type `yes` when prompted. Three confident readings are required before forward
motion. Failure to acquire produces `ACQUISITION_FAILED`; line loss stops and
latches `LINE_LOST`. The three-second total includes acquisition. A successful
bounded test ends with `MAX_DURATION` and stops all four wheels. At five counts
the motors may not turn, even with raised wheels. If so, stop here and report
the result; gain changes cannot overcome a fixed five-count motor limit.
Physical gains have not been calibrated here. The control CSV contains observed
poll/control rates, deadlines, sensor age, PID terms, and PWM commands; requested
300/100 Hz rates do not guarantee those rates on the Pi.

Reposition by hand for the other boundary and change to `--edge black-right`.
The separate test file uses the **same CLI safety session**, not another driver:

```bash
python ~/waferbot-test/app/scripts/test_line_follow.py --physical \
  --edge black-left --duration 3 \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/.config/waferbot/nav.pid-first-run.json
```

If the straight run passes but sharp corners are missed, use the tight-corner
profile for another short test. Copy it to an operator-owned file and set
`follow.controller_enabled` to `true` only after reviewing its parameters. It
keeps the five-count hardware cap, uses base speed 5, reduces forward command
to 1 at full steering demand, and allows up to 60 PWM/s correction slew.
Recovery remains disabled, so losing
the boundary stops the robot. Place the sensor bar just before the corner so
the bend enters view during this short run:

```bash
cp ~/waferbot-test/app/config/nav.windy-first-run.json ~/.config/waferbot/nav.pid-windy.json
python - <<'PY'
from pathlib import Path
import json
path = Path.home() / '.config/waferbot/nav.pid-windy.json'
data = json.loads(path.read_text())
data['follow']['controller_enabled'] = True
path.write_text(json.dumps(data, indent=2) + '\n')
PY
waferbot follow --physical --edge black-left --duration 3 \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/.config/waferbot/nav.pid-windy.json \
  --control-log-csv ~/waferbot-logs/tight-corner-control.csv
```

Use this only after confirming the correct edge is centered and wheel directions
are right. At a large error the differential mixer can command one side forward
and the other backward, with each wheel still bounded to ±5. If turns fail in
only one direction, correct wheel inversion or alignment before changing gains.
If the robot loses a square corner, inspect the CSV and tape geometry before
increasing gains: a four-channel front sensor may lose a mathematically sharp
90° edge before it can pivot. The current synthetic 80.5° rising kink stays
on the edge, but this does not validate a physical square corner. A rounded
tape corner or wider visible boundary can help. For oscillation on mild curves,
lower `kp_pwm_per_mm` or increase `velocity_filter_tau_s`. Keep recovery
disabled while tuning. This is a full PID implementation with initial Ki=0.

## 6. Stop controls

Ctrl+C stops the current session. From a second SSH terminal as the same user:

```bash
~/.venvs/waferbot/bin/waferbot stop --physical \
  --config ~/.config/waferbot/robot.first-run.json
~/.venvs/waferbot/bin/waferbot stop --status
```

This latches a stop and writes stop commands. Further movement is refused until
the original session exits and you explicitly clear it:

```bash
~/.venvs/waferbot/bin/waferbot stop --clear
```

All terminals must use the same runtime directory (default `~/.cache/waferbot`).
If you override `--runtime-dir` or `WAFERBOT_RUNTIME_DIR`, use that same setting
for follow and stop. The software cannot cut power: a killed process or wedged
I2C bus may leave the controller holding its last command. Use the physical
switch if the robot does not stop.

## 7. Simulation and tuning

On the Mac (or in the installed bundle's `app/` directory):

```bash
python3 scripts/test_line_follow.py --edge both
python3 scripts/test_line_follow.py --edge both --offset-mm 10 --heading-deg 5
python3 scripts/test_line_follow.py --edge both \
  --config config/robot.first-run.json \
  --nav-config config/nav.windy-first-run.json
```

This drives the actual follower against synthetic tape geometry and wheel
kinematics. It tests feedback direction; the five-count profile may not reach
the simulator's convergence threshold within its ten-second limit. It does not
measure real friction, slip, sensor height or lighting. For later tuning, copy
`config/nav.windy-first-run.json` to a new operator-owned file and pass it
explicitly. Keep recovery disabled until the short straight runs behave. The
separate tuned template enables bounded recovery.

| Parameter | First run | Winding profile |
| --- | ---: | ---: |
| base_speed (PWM) | 4 | 5 |
| kp_pwm_per_mm | 0.307692 | 0.769231 |
| ki_pwm_per_mm_s | 0 | 0 |
| kd_pwm_s_per_mm | 0.009231 | 0.015385 |
| max_correction | 4 | 5 |
| max_correction_slew_pwm_per_s | 20 | 60 |
| min_curve_speed_factor | 0.45 | 0.20 |
| curve_slowdown_start | 0.20 | 0.0 |
| velocity_filter_tau_s | 0.05 | 0.05 |
| recovery_enabled | false | false |

See [PID_CONTROL.md](PID_CONTROL.md) for equations, coordinate signs, timing,
logging, replay and simulation commands.

The deterministic simulation tracks an 80.5° rising kink on either edge with
this profile; the previous profile loses it. The model assumes ideal sensing
and wheel response. This is evidence for the controller change, not proof that
the physical robot can take a perfectly square 90° corner. Review the CSV
before longer runs. To copy telemetry back to the Mac:

```bash
mkdir -p logs
scp pi@waferbot.local:waferbot-logs/first-follow.csv logs/
```

## 8. Crossing and graph execution after measurements

Use a separate calibrated `~/.config/waferbot/nav.json`. Record the real tape
width, sensor spacing/footprint, robot width, crossing distance, margin, lateral
clearance, speed, polling rate and required confirmations in `geometry`, then
set `measured: true`. Record `switch.max_travel_m` and a conservative measured
`counts_to_mps` (see `waferbot calibrate speed --instructions`). Neither the
example graph nor first-run template supplies invented physical measurements.

```bash
waferbot calibrate crossing --physical \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/.config/waferbot/nav.json \
  --angles 12,15,20,25 --attempts 3 --csv ~/waferbot-logs/crossing.csv

waferbot switch --physical --from-edge black-left --to-edge black-right \
  --location B2 --route-id manual --authorize \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/.config/waferbot/nav.json
```

The sweep asks you to reposition at the source before every attempt. Switching
requires explicit location authorization and destination confirmation; observing
the other edge is not localization. Movement budgets exclude stopped prompts,
and fresh edge verification is required after prompts before moving again.

Inspect the illustrative route without moving anything:

```bash
waferbot plan --start A+ --goal D- --via B-
waferbot plan --start A+ --goal D- --via B- --algorithm astar
waferbot execute --start A+ --goal D- --via B- --dry-run
```

Populate your own `~/.config/waferbot/track.json` with measured topology,
coordinates, absolute chassis headings (degrees CCW from +x), edge-side mapping,
speeds and markers; only then set `is_example: false`, `physical_validated:
true`. Positive/negative labels do not imply sensor polarity. Reverse FOLLOW
is explicitly unsupported and refused before movement; use measured forward
connections. DOCK can reverse. TURN uses absolute heading change.

```bash
waferbot execute --physical --start A+ --goal D- --via B- \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/.config/waferbot/nav.json \
  --map ~/.config/waferbot/track.json --localizer manual
```

Manual localization pauses with motors stopped and requires looking at the
track, including heading when prompted. A future real marker reader can use
`--localizer marker --marker-reader module:factory`; mock/scripted location
claims are refused in physical mode. Elapsed time does not confirm arrival.

## Fresh SD card (only for a reinstall)

Use Raspberry Pi Imager, select Raspberry Pi OS 64-bit, configure hostname
`waferbot`, user `pi`, SSH and network credentials, and flash the selected card
manually. Boot, connect over SSH, then follow the steps above. Nothing in these
scripts erases storage. Full setup references and the configurable SSH deployment
script are in `README.md` and `deployment/README.md`.
