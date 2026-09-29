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
rsync -az --partial dist/waferbot-offline-0.3.0/ pi@waferbot.local:waferbot-test/
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

Expect `waferbot 0.3.0`. The Python install verifies the manifest, installs with
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
within reach. This command uses 20 Hz, base PWM 4, a five-count wheel limit from
your first-run robot config, stationary acquisition, and **recovery disabled**:

```bash
waferbot follow --physical --edge black-left --duration 3 \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/waferbot-test/app/config/nav.first-run.json \
  --log-csv ~/waferbot-logs/first-follow.csv
```

Type `yes` when prompted. Three confident readings are required before forward
motion. Failure to acquire produces `ACQUISITION_FAILED`; line loss stops and
latches `LINE_LOST`. The three-second total includes acquisition. A successful
bounded test ends with `MAX_DURATION` and stops all four wheels. At five counts
the motors may not turn, even with raised wheels. If so, stop here and report
the result; gain changes cannot overcome a fixed five-count motor limit.
Physical gains have not been calibrated here.

Reposition by hand for the other boundary and change to `--edge black-right`.
The separate test file uses the **same CLI safety session**, not another driver:

```bash
python ~/waferbot-test/app/scripts/test_line_follow.py --physical \
  --edge black-left --duration 3 \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/waferbot-test/app/config/nav.first-run.json
```

If the straight run passes but winding sections are missed, use the curve
profile for another short test. It keeps the five-count hardware cap, uses base
speed 5, slows further when steering demand rises, permits a larger
differential correction, and still stops instead of searching after line loss:

```bash
waferbot follow --physical --edge black-left --duration 3 \
  --config ~/.config/waferbot/robot.first-run.json \
  --nav-config ~/waferbot-test/app/config/nav.windy-first-run.json \
  --log-csv ~/waferbot-logs/windy-follow.csv
```

Use this only after confirming the correct edge is centered and wheel directions
are right. If turns fail in only one direction, correct wheel inversion or
alignment before changing gains. If both directions fail at the same bend,
reduce `base_speed` from 5 to 4, then 3 if needed. For oscillation on mild
curves, reduce `kp` by 0.25 or increase `derivative_filter_alpha` toward 0.65.
If response is late without oscillation, increase `kp` by 0.25 at a time. Keep
`max_correction` at or below 5 with this wheel limit, and keep recovery disabled
while tuning. This controller uses proportional and derivative terms; there is
no integral term to tune.

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
| kp (counts per sensor pitch) | 2.0 | 2.5 |
| kd (counts per pitch/second) | 0.06 | 0.08 |
| max_correction | 4 | 5 |
| max_correction_delta (per cycle) | 1 | 1 |
| min_curve_speed_factor | 0.45 | 0.40 |
| curve_slowdown_start | 0.20 | 0.15 |
| derivative_filter_alpha | 0.5 | 0.5 |
| recovery_enabled | false | false |

Change one setting at a time. Increase proportional gain gradually if correction
is too weak; reduce speed/gain if oscillation grows. Review the CSV before
longer runs. To copy telemetry back to the Mac:

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
