# Stationary AprilTag test on the Raspbot V2

`waferbot tags` reads the USB camera through OpenCV `VideoCapture(0)`, matching
the camera path used by the existing Yahboom demos. It recognizes the
`tag36h11` family. It never opens the motor controller or arms the robot.
It prints a newly visible tag ID once, appends it to CSV if requested, briefly
shows `APRILTAG DETECTED:` and the ID on the stock Yahboom OLED, then restores
the command's normal `Waferbot` / `Ready` screen.

The repository contains no earlier AprilTag reader or OLED driver. A vendored
Yahboom display demo imports `Yahboom_OLED` from
`/home/pi/software/oled_yahboom/yahboom_oled.py` and uses `clear`, `add_line`,
and `refresh`. This command uses that installed driver without copying or
modifying it. If its path differs, pass `--oled-driver PATH`. The stock driver
does not expose a way to read another application's previous screen; run this
diagnostic while it owns the display. Pass `--restore-line TEXT` twice to set
the two normal lines it restores.

After installing the 0.3.0 offline bundle as in [ROBOT_TESTING.md](ROBOT_TESTING.md),
install the optional OpenCV camera dependency on the Pi while online:

```bash
~/.venvs/waferbot/bin/python -m pip install --only-binary=:all: \
  'opencv-contrib-python-headless>=4.7,<5'
~/.venvs/waferbot/bin/python -c 'import cv2; print(cv2.__version__, cv2.aruco.DICT_APRILTAG_36h11)'
test -f /home/pi/software/oled_yahboom/yahboom_oled.py
```

OpenCV is optional and is **not** in the base offline bundle. The `--only-binary`
flag prevents an accidental hours-long source build if a compatible ARM64 wheel
is unavailable. If the driver path check fails, locate the stock OLED driver
on the Pi and pass its path explicitly, or first test the camera with
`--display none`. Do not run another OLED-writing program concurrently.

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
