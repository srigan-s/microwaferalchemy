# Test one tape edge without moving the robot

Use `scripts/test_line_edge.py`. It is a standalone sensor diagnostic, independent
of the navigation package, map, calibration flags and motor controller code. It
only reads the documented line-sensor register; it never commands a motor.

Keep the robot stationary. View the sensors from behind the robot: S1 is leftmost,
then S2, S3, and S4. Put a tape boundary between S2 and S3. For BLACK_LEFT, S2
should be over black tape and S3 over white floor. For BLACK_RIGHT, reverse them.
The printed normalized readings use **black = 1**, **white = 0**.

## Install and run on the Raspberry Pi

From this project directory on the Pi:

```bash
sudo raspi-config nonint do_i2c 0
sudo apt-get install -y python3-venv
python3 -m venv .venv-line
.venv-line/bin/python -m pip install smbus2
.venv-line/bin/python scripts/test_line_edge.py --edge black-left
```

If `/dev/i2c-1` does not appear, reboot the Pi. If permission is denied, run
`sudo usermod -aG i2c "$(id -un)"`, then log out and reconnect. Raspberry Pi OS
must have I2C enabled and the Raspbot controller must be powered.

For the other boundary:

```bash
.venv-line/bin/python scripts/test_line_edge.py --edge black-right
```

Stop with **Ctrl+C**. To collect only 100 readings, add `--samples 100`.

`EDGE_OK` means the chosen S2/S3 pattern was read on three consecutive samples.
`SETTLING` means it has not yet repeated enough times. `OTHER_EDGE` means the
opposite boundary. `BOTH_BLACK` and `BOTH_WHITE` are ambiguous: shift the tape
slightly by hand until just one middle sensor sees black. This checks edge
sensing only; it does not drive or follow the line, or prove robot localization.

## Copy only the test file from the Mac

```bash
ssh pi@waferbot.local 'mkdir -p ~/microwaferalchemy/scripts'
scp scripts/test_line_edge.py pi@waferbot.local:~/microwaferalchemy/scripts/
ssh pi@waferbot.local
cd ~/microwaferalchemy
# Run the install/run commands above.
```

Change `pi` and `waferbot.local` if you configured a different user/hostname.

## Confirm channel order and polarity

Existing vendor notebooks suggest raw **0 = black**, and physical S1..S4 uses
bits **2,3,1,0** of the byte read from I2C bus **1**, address **0x2B**, register
**0x0A**. This still needs checking on your board. Cover one sensor at a time with
black tape and check that the corresponding normalized column becomes 1.

If your measured wiring uses another order, pass e.g. `--bits 3,2,1,0`. If raw
black is 1 on your unit, pass `--black-value 1`. Do not change these simply to
force EDGE_OK; establish the individual sensor responses first.

On the Mac, a synthetic demonstration requires no dependencies:

```bash
python3 scripts/test_line_edge.py --demo --samples 12
python3 -m unittest discover -s tests -p test_line_edge_standalone.py -v
```

No physical hardware test has been performed during development.
