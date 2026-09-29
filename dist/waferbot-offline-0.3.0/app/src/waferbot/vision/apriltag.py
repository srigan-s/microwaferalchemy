"""Stationary tag36h11 recognition using the Yahboom demos' OpenCV camera path.

This module never opens the motor controller. The string ``marker_id`` is ready
for an explicit map from tag IDs to navigation nodes in a future localizer.
"""

from __future__ import annotations

import csv
import importlib
import importlib.util
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Protocol

from ..errors import WaferbotError

FAMILY = "tag36h11"
DEFAULT_OLED_DRIVER = Path("/home/pi/software/oled_yahboom/yahboom_oled.py")


@dataclass(frozen=True)
class AprilTagDetection:
    tag_id: int
    timestamp: float
    corners: tuple[tuple[float, float], ...]
    family: str = FAMILY

    @property
    def marker_id(self) -> str:
        return str(self.tag_id)


def _opencv(module: Any | None = None) -> Any:
    if module is not None:
        return module
    try:
        return importlib.import_module("cv2")
    except ImportError as exc:
        raise WaferbotError(
            "OpenCV with ArUco is required for tag detection; install "
            "opencv-contrib-python-headless in the waferbot virtual environment"
        ) from exc


class OpenCvAprilTagDetector:
    """Detect IDs and image corners; no invented pose or confidence estimate."""

    def __init__(self, *, cv: Any | None = None, clock: Callable[[], float] = time.monotonic):
        self.cv = _opencv(cv)
        aruco = getattr(self.cv, "aruco", None)
        if aruco is None or not hasattr(aruco, "DICT_APRILTAG_36h11"):
            raise WaferbotError(
                "this OpenCV installation lacks cv2.aruco.DICT_APRILTAG_36h11; "
                "install an ArUco-enabled OpenCV build"
            )
        self._clock = clock
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        if hasattr(aruco, "ArucoDetector"):
            self._detect = aruco.ArucoDetector(dictionary).detectMarkers
        elif hasattr(aruco, "detectMarkers"):
            self._detect = lambda frame: aruco.detectMarkers(frame, dictionary)
        else:
            raise WaferbotError("this OpenCV build has no ArUco marker detector")

    def detect(self, frame: Any) -> tuple[AprilTagDetection, ...]:
        corners, ids, _rejected = self._detect(frame)
        if ids is None:
            return ()
        timestamp = self._clock()
        found = []
        for raw_id, raw_corners in zip(ids, corners):
            points = raw_corners.reshape(-1, 2) if hasattr(raw_corners, "reshape") else raw_corners
            found.append(
                AprilTagDetection(
                    tag_id=int(raw_id[0] if hasattr(raw_id, "__len__") else raw_id),
                    timestamp=timestamp,
                    corners=tuple((float(x), float(y)) for x, y in points),
                )
            )
        return tuple(sorted(found, key=lambda tag: tag.tag_id))


class OpenCvCamera:
    """OpenCV VideoCapture(0), matching the existing Yahboom USB camera demos."""

    def __init__(self, index: int = 0, *, cv: Any | None = None):
        if index < 0:
            raise ValueError("camera index must be nonnegative")
        self.cv = _opencv(cv)
        self.capture = self.cv.VideoCapture(index)
        if not self.capture.isOpened():
            self.capture.release()
            raise WaferbotError(f"camera {index} could not be opened")

    def read(self) -> Any:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            raise WaferbotError("camera did not return a frame")
        return frame

    def close(self) -> None:
        self.capture.release()

    def __enter__(self) -> "OpenCvCamera":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class TagDisplay(Protocol):
    def show(self, tag_id: int) -> None: ...
    def restore(self) -> None: ...


class NoDisplay:
    def show(self, tag_id: int) -> None:
        return None

    def restore(self) -> None:
        return None


def _load_oled_driver(path: Path) -> Any:
    if not path.is_file():
        raise WaferbotError(
            f"Yahboom OLED driver not found at {path}; pass --oled-driver or "
            "use --display none for a camera-only diagnostic"
        )
    spec = importlib.util.spec_from_file_location("yahboom_oled", path)
    if spec is None or spec.loader is None:
        raise WaferbotError(f"cannot load Yahboom OLED driver from {path}")
    module = importlib.util.module_from_spec(spec)
    # The stock driver may import sibling modules from its own directory.
    sys.path.insert(0, str(path.parent))
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        raise WaferbotError(f"Yahboom OLED driver failed to load: {exc}") from exc
    finally:
        sys.path.remove(str(path.parent))
    return module


class YahboomOledDisplay:
    """Brief overlay using the API referenced by the vendored Yahboom demo.

    The command owns the display while running and restores its configured
    normal lines after the notice. The external driver offers no readback of
    another application's prior screen.
    """

    def __init__(
        self,
        *,
        driver_path: str | Path = DEFAULT_OLED_DRIVER,
        normal_lines: tuple[str, str] = ("Waferbot", "Ready"),
        driver: Any | None = None,
    ) -> None:
        if len(normal_lines) != 2:
            raise ValueError("normal_lines must contain two display lines")
        module = driver or _load_oled_driver(Path(driver_path))
        factory = getattr(module, "Yahboom_OLED", None)
        if factory is None:
            raise WaferbotError("Yahboom OLED driver has no Yahboom_OLED class")
        try:
            self.oled = factory(debug=False)
            self.oled.init_oled_process()
        except Exception as exc:
            raise WaferbotError(f"Yahboom OLED initialization failed: {exc}") from exc
        self.normal_lines = normal_lines
        self.restore()

    def _draw(self, first: str, second: str) -> None:
        self.oled.clear()
        self.oled.add_line(first, 1)
        self.oled.add_line(second, 3)
        self.oled.refresh()

    def show(self, tag_id: int) -> None:
        self._draw("APRILTAG DETECTED:", str(tag_id))

    def restore(self) -> None:
        self._draw(*self.normal_lines)


def scan_stationary(
    camera: Any,
    detector: Any,
    display: TagDisplay,
    *,
    duration_s: float = 10.0,
    notice_s: float = 1.5,
    max_frames: int | None = None,
    csv_path: str | Path | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    emit: Callable[[str], None] = print,
) -> int:
    """Read only the camera and display; report each newly visible tag once."""
    if (
        not math.isfinite(duration_s) or duration_s <= 0
        or not math.isfinite(notice_s) or notice_s <= 0
        or max_frames is not None and max_frames < 1
    ):
        raise ValueError("duration, notice time and max frames must be positive")
    handle = None
    writer = None
    seen: set[int] = set()
    displayed_until = 0.0
    displayed = False
    count = 0
    frames = 0
    deadline = clock() + duration_s
    try:
        if csv_path is not None:
            path = Path(csv_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            new_file = not path.exists() or path.stat().st_size == 0
            handle = path.open("a", newline="", encoding="utf-8")
            writer = csv.writer(handle)
            if new_file:
                writer.writerow(("timestamp_utc", "family", "tag_id", "marker_id"))
        while clock() < deadline and (max_frames is None or frames < max_frames):
            detections = detector.detect(camera.read())
            frames += 1
            current = {d.tag_id for d in detections}
            new = sorted(current - seen)
            for tag_id in new:
                emit(f"APRILTAG DETECTED: {tag_id}")
                if writer is not None:
                    writer.writerow((datetime.now(timezone.utc).isoformat(), FAMILY, tag_id, str(tag_id)))
                    handle.flush()
                count += 1
            if new:
                display.show(new[0])
                displayed_until = clock() + notice_s
                displayed = True
            elif displayed and clock() >= displayed_until:
                display.restore()
                displayed = False
            seen = current
    finally:
        try:
            try:
                if displayed and clock() < displayed_until:
                    sleep(displayed_until - clock())
            finally:
                display.restore()
        finally:
            if handle is not None:
                handle.close()
            camera.close()
    return count


__all__ = [
    "AprilTagDetection",
    "OpenCvAprilTagDetector",
    "OpenCvCamera",
    "YahboomOledDisplay",
    "NoDisplay",
    "scan_stationary",
]
