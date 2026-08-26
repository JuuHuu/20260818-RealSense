"""OpenCV VideoCapture wrapper for an ordinary RGB camera."""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

from .camera import CameraFrame
from .io_utils import load_yaml


def load_camera_intrinsics(
    path: str | Path, image_size: tuple[int, int]
) -> tuple[np.ndarray, np.ndarray]:
    """Load and validate pinhole intrinsics for an exact image size."""

    data = load_yaml(path)
    calibrated_size = (int(data["image_width"]), int(data["image_height"]))
    if calibrated_size != image_size:
        raise ValueError(
            f"camera calibration {path} is for {calibrated_size[0]}x{calibrated_size[1]}, "
            f"but the camera produced {image_size[0]}x{image_size[1]}; recalibrate at the capture resolution"
        )
    camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
    distortion = np.asarray(data["distortion_coefficients"], dtype=np.float64).reshape(-1, 1)
    if camera_matrix.shape != (3, 3) or not np.all(np.isfinite(camera_matrix)):
        raise ValueError(f"{path} has an invalid camera_matrix")
    if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
        raise ValueError(f"{path} has non-positive focal lengths")
    if distortion.size not in {4, 5, 8, 12, 14} or not np.all(np.isfinite(distortion)):
        raise ValueError(f"{path} has invalid distortion_coefficients")
    return camera_matrix, distortion


class OpenCVColorCamera:
    def __init__(
        self,
        device: int | str,
        width: int,
        height: int,
        fps: int,
        intrinsics_file: str | Path | None = None,
        fourcc: str | None = None,
    ):
        self.device = int(device) if isinstance(device, str) and device.isdigit() else device
        self.requested_width = int(width)
        self.requested_height = int(height)
        self.requested_fps = int(fps)
        self.intrinsics_file = Path(intrinsics_file) if intrinsics_file else None
        self.fourcc = fourcc
        self.capture = None
        self.camera_matrix = None
        self.distortion = None
        self.image_size = None
        self._pending_frame = None

    def start(self) -> None:
        if self.capture is not None:
            raise RuntimeError("camera is already started")
        capture = cv2.VideoCapture(self.device)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open RGB camera device {self.device!r}")
        try:
            if self.fourcc:
                if len(self.fourcc) != 4:
                    raise ValueError("camera.fourcc must contain exactly four characters")
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.requested_width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.requested_height)
            capture.set(cv2.CAP_PROP_FPS, self.requested_fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, image = capture.read()
            if not ok or image is None:
                raise RuntimeError(f"RGB camera device {self.device!r} returned no frame")
            self.image_size = (int(image.shape[1]), int(image.shape[0]))
            if self.intrinsics_file is not None:
                self.camera_matrix, self.distortion = load_camera_intrinsics(
                    self.intrinsics_file, self.image_size
                )
            self.capture = capture
            self._pending_frame = CameraFrame(image, time.time())
        except Exception:
            capture.release()
            raise

    def read(self) -> CameraFrame:
        if self.capture is None:
            raise RuntimeError("camera has not been started")
        if self._pending_frame is not None:
            frame = self._pending_frame
            self._pending_frame = None
            return frame
        ok, image = self.capture.read()
        if not ok or image is None:
            raise RuntimeError(f"RGB camera device {self.device!r} returned no frame")
        return CameraFrame(image, time.time())

    def stop(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self._pending_frame = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
