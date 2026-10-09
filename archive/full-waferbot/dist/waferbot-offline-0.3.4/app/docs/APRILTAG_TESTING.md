# Stationary AprilTag test on the Raspbot V2

`waferbot tags` reads the USB camera through OpenCV `VideoCapture(0)`, matching
the camera path used by the existing Yahboom demos. It recognizes the
`tag36h11` family. It never opens the motor controller or arms the robot.
It prints a newly visible tag ID once, appends it to CSV if requested, briefly
shows `APRILTAG DETECTED:` and the ID on the stock Yahboom OLED, then restores
the command's normal `Waferbot` / `Ready` screen.

The [Yahboom key-reading notebook](../vendor/yahboom/project_demo/03.Basic_car_course/2.Read%20the%20key%20value.ipynb)
shows the stock `Yahboom_OLED` methods but imports its driver from an external
file absent on this Pi. Waferbot now includes a compatible `Yahboom_OLED`
implementation for a 128×32 SSD1306 screen. It uses the already bundled
`smbus2` dependency and opens only the OLED's I²C address, never the motor
controller's address. `init_oled_process()` retains the Yahboom API name but
does not launch a background process. An external driver remains available as
an explicit `--oled-driver PATH` override.

Your `i2cdetect -y 1` check showed an OLED at `0x3c`, matching the packaged
default (bus 1, address `0x3c`). If the display is moved, use `--oled-bus N`
or `--oled-address 0xNN`. The driver cannot read another application's prior
screen, so run this diagnostic while it owns the display. Pass
`--restore-line TEXT` twice to choose the two normal lines it restores.

After installing the 0.3.4 offline bundle as in [ROBOT_TESTING.md](ROBOT_TESTING.md),
test the OLED **without opening the camera or motors**:

```bash
~/.venvs/waferbot/bin/waferbot oled-test --duration 3
```

It writes `Waferbot` on the first row and `Ready` on the third row, then
releases the I²C bus while leaving that screen visible. If this fails, check
that `i2cdetect -y 1` still shows `3c` and that the `pi` user can open
`/dev/i2c-1`. Then install the optional OpenCV camera dependency while online:

```bash
~/.venvs/waferbot/bin/python -m pip install --only-binary=:all: \
  'opencv-contrib-python-headless>=4.7,<5'
~/.venvs/waferbot/bin/python -c 'import cv2; print(cv2.__version__, cv2.aruco.DICT_APRILTAG_36h11)'
```

OpenCV is optional and is **not** in the base offline bundle. The `--only-binary`
flag prevents an accidental hours-long source build if a compatible ARM64 wheel
is unavailable. `--display none` still tests the camera without touching the
OLED. Do not run another OLED-writing program concurrently.

To make a test marker (ID 42) after OpenCV is installed:

```bash
~/.venvs/waferbot/bin/python - <<'PY'
import cv2
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
tag = cv2.aruco.generateImageMarker(dictionary, 42, 480)
tag = cv2.copyMakeBorder(tag, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
cv2.imwrite('tag36h11-42.png', tag)
PY
```

Print the image or show it on another screen, with the entire black border and
white margin visible. Put the robot stationary, aim its camera at the marker,
and run:

```bash
mkdir -p ~/waferbot-logs
~/.venvs/waferbot/bin/waferbot tags --camera-index 0 --duration 10 \
  --display yahboom --log-csv ~/waferbot-logs/apriltags.csv
```

Expected console output includes `APRILTAG DETECTED: 42`; the OLED briefly
shows the same ID and returns to `Waferbot` / `Ready`. The CSV stores UTC time,
family, numeric ID, and string `marker_id`. For camera-only diagnosis, add
`--display none`. If camera 0 cannot open, check the attached camera device and
try another `--camera-index`. Ctrl+C releases the camera and restores the OLED.

`AprilTagDetection.marker_id` is the string form of the numeric tag ID, ready
to match a future measured `MapNode.marker_id`. This diagnostic does not claim
a navigation node from a tag, estimate pose, or move the chassis.
