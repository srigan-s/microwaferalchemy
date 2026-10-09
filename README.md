# Waferbot PID line follower

This folder contains only the physical IR → position → PID → lateral wheel-control path. The older route planning (Dijkstra/A*), AprilTag, calibration experiments, simulations, vendor reference, and previous deployment are preserved in [`archive/full-waferbot/`](archive/full-waferbot/).

Start here:

| File | What it does |
| --- | --- |
| [`src/waferbot/sensing/follower.py`](src/waferbot/sensing/follower.py) | Bounded physical control loop and four-wheel mixer |
| [`src/waferbot/sensing/poller.py`](src/waferbot/sensing/poller.py) | Keeps the newest 300 Hz IR sample |
| [`src/waferbot/sensing/position.py`](src/waferbot/sensing/position.py) | Converts the four IR bits to position in millimetres |
| [`src/waferbot/sensing/pid.py`](src/waferbot/sensing/pid.py) | PID equations and anti-windup |
| [`src/waferbot/pidconfig.py`](src/waferbot/pidconfig.py) | PID settings and validation |
| [`src/waferbot/robot.py`](src/waferbot/robot.py) | Safe motor and sensor interface; watchdog and stop checks |
| [`config/pid.first-run.json`](config/pid.first-run.json) | Slow, disabled-by-default first-run settings |

Motor commands are integer PWM, capped at ±5. A background reader requests IR samples at 300 Hz, while PID commands run at 100 Hz; observed rates on Raspberry Pi OS can vary. The default base is 4 PWM and the mixer rounds each wheel command independently; there is no fractional PWM accumulation. The follower uses the selected black-left or black-right transition, with `1100` giving 0 mm and `1000` giving +3.25 mm for black-left. It stops on stale/invalid sensor readings, a stop request, a fault, or the duration limit. Recovery manoeuvres and route navigation are not in this small project.

## Install on the Pi

From this repository on your Mac, copy only the active project (not the archive):

```bash
rsync -az --exclude archive/ --exclude .git/ --exclude __pycache__/ ./ pi@waferbot.local:~/waferbot-pid/
ssh pi@waferbot.local
cd ~/waferbot-pid
python3 -m venv ~/.venvs/waferbot-pid
~/.venvs/waferbot-pid/bin/pip install --no-index --force-reinstall --find-links wheels waferbot smbus2
source ~/.venvs/waferbot-pid/bin/activate
waferbot --version
```

Check sensors while stationary:

```bash
waferbot sensors --physical --robot-config config/robot.first-run.json
```

Copy and inspect `config/pid.first-run.json` and `config/robot.first-run.json` before moving the robot. Set `enabled` to `true` in your *copy* of the PID config only after confirming sensor order and motor directions. To run a supervised three-second black-left test:

```bash
waferbot follow --physical --robot-config config/robot.first-run.json --pid-config config/pid.local.json --edge black-left --duration 3 --control-log-csv ~/waferbot-logs/pid-first-run.csv
```

The command asks you to type `yes` before motion. In another SSH session, stop it with:

```bash
waferbot stop --robot-config ~/waferbot-pid/config/robot.first-run.json
```

After a stop, `waferbot clear-stop` works only while no motion session owns the robot. Keep the first test over a short, clear section of tape and stay ready to cut motor power. There is no automatic motion at startup.

To inspect a controller CSV locally, run `python3 plot_pid.py <csv_file>` after installing matplotlib on your development computer. The full earlier project remains in the archive if you need a feature back.
