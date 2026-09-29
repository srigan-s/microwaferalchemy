"""AprilTag camera/OLED diagnostics without importing physical hardware."""

from __future__ import annotations

import csv
from types import SimpleNamespace

import pytest

from waferbot.cli import main
from waferbot.errors import WaferbotError
from waferbot.vision import apriltag


class FakeCv:
    def __init__(self):
        self.released = False
        outer = self

        class Aruco:
            DICT_APRILTAG_36h11 = 20

            @staticmethod
            def getPredefinedDictionary(value):
                assert value == 20
                return "tag36h11"

            class ArucoDetector:
                def __init__(self, dictionary):
                    assert dictionary == "tag36h11"

                @staticmethod
                def detectMarkers(frame):
                    assert frame == "frame"
                    return ([[(1, 2), (3, 4), (5, 6), (7, 8)]], [[42]], [])

        self.aruco = Aruco

    def VideoCapture(self, index):
        assert index == 0
        outer = self

        class Capture:
            def isOpened(self):
                return True

            def read(self):
                return True, "frame"

            def release(self):
                outer.released = True

        return Capture()


def test_opencv_adapter_reads_tag36h11_id_and_releases_camera():
    cv = FakeCv()
    detector = apriltag.OpenCvAprilTagDetector(cv=cv, clock=lambda: 12.0)
    camera = apriltag.OpenCvCamera(cv=cv)
    with camera:
        found = detector.detect(camera.read())
    assert len(found) == 1
    assert found[0].tag_id == 42
    assert found[0].marker_id == "42"
    assert found[0].timestamp == 12.0
    assert found[0].corners == ((1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (7.0, 8.0))
    assert cv.released


def test_real_opencv_decodes_generated_tag36h11_marker():
    cv = pytest.importorskip("cv2")
    aruco = getattr(cv, "aruco", None)
    if aruco is None or not hasattr(aruco, "generateImageMarker"):
        pytest.skip("ArUco-enabled OpenCV is not installed")
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    tag = aruco.generateImageMarker(dictionary, 42, 240)
    frame = cv.copyMakeBorder(tag, 80, 80, 80, 80, cv.BORDER_CONSTANT, value=255)
    found = apriltag.OpenCvAprilTagDetector(cv=cv).detect(frame)
    assert [item.tag_id for item in found] == [42]


def test_detector_refuses_opencv_without_apriltag_dictionary():
    with pytest.raises(WaferbotError, match="DICT_APRILTAG_36h11"):
        apriltag.OpenCvAprilTagDetector(cv=SimpleNamespace(aruco=SimpleNamespace()))


def test_camera_open_failure_releases_capture():
    class Cv:
        def __init__(self):
            self.released = False

        def VideoCapture(self, index):
            owner = self

            class Capture:
                def isOpened(self):
                    return False

                def release(self):
                    owner.released = True

            return Capture()

    cv = Cv()
    with pytest.raises(WaferbotError, match="could not be opened"):
        apriltag.OpenCvCamera(cv=cv)
    assert cv.released


def test_yahboom_oled_notice_then_normal_screen():
    class Oled:
        def __init__(self, *, debug):
            assert debug is False
            self.screens = []
            self.lines = []

        def init_oled_process(self):
            pass

        def clear(self):
            self.lines = []

        def add_line(self, text, row):
            self.lines.append((row, text))

        def refresh(self):
            self.screens.append(tuple(self.lines))

    display = apriltag.YahboomOledDisplay(driver=SimpleNamespace(Yahboom_OLED=Oled))
    display.show(42)
    display.restore()
    assert display.oled.screens == [
        ((1, "Waferbot"), (3, "Ready")),
        ((1, "APRILTAG DETECTED:"), (3, "42")),
        ((1, "Waferbot"), (3, "Ready")),
    ]


def test_stationary_scan_logs_new_ids_and_restores_display(tmp_path):
    tick = [0.0]
    screens = []
    messages = []

    class Camera:
        closed = False

        def read(self):
            tick[0] += 0.25
            return "frame"

        def close(self):
            self.closed = True

    class Detector:
        samples = iter(((7,), (7,), (), (7,)))

        def detect(self, frame):
            return tuple(SimpleNamespace(tag_id=value) for value in next(self.samples))

    class Display:
        def show(self, tag_id):
            screens.append(("tag", tag_id))

        def restore(self):
            screens.append(("normal", None))

    camera = Camera()
    path = tmp_path / "tags.csv"
    count = apriltag.scan_stationary(
        camera, Detector(), Display(), duration_s=10, max_frames=4,
        notice_s=0.5, csv_path=path, clock=lambda: tick[0],
        sleep=lambda duration: tick.__setitem__(0, tick[0] + duration),
        emit=messages.append,
    )
    assert count == 2
    assert messages == ["APRILTAG DETECTED: 7"] * 2
    assert screens == [("tag", 7), ("normal", None), ("tag", 7), ("normal", None)]
    assert camera.closed
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert [row["tag_id"] for row in rows] == ["7", "7"]
    assert all(row["family"] == "tag36h11" for row in rows)


def test_stationary_scan_restores_screen_and_closes_camera_on_read_failure():
    events = []

    class Camera:
        def read(self):
            raise WaferbotError("camera did not return a frame")

        def close(self):
            events.append("camera closed")

    class Display:
        def restore(self):
            events.append("normal screen")

    class Detector:
        def detect(self, frame):
            pytest.fail("camera read should fail first")

    with pytest.raises(WaferbotError, match="camera did not return"):
        apriltag.scan_stationary(Camera(), Detector(), Display())
    assert events == ["normal screen", "camera closed"]


def test_tags_cli_never_builds_motor_session(monkeypatch, capsys):
    import waferbot.cli as cli

    monkeypatch.setattr(cli, "_build_session", lambda *_a, **_k: pytest.fail("motor session opened"))

    class Camera:
        def __init__(self, index):
            assert index == 0

        def read(self):
            return "frame"

        def close(self):
            pass

    class Detector:
        def detect(self, frame):
            return (SimpleNamespace(tag_id=13),)

    monkeypatch.setattr(apriltag, "OpenCvCamera", Camera)
    monkeypatch.setattr(apriltag, "OpenCvAprilTagDetector", Detector)
    assert main(["tags", "--display", "none", "--max-frames", "1"]) == 0
    assert "APRILTAG DETECTED: 13" in capsys.readouterr().out
