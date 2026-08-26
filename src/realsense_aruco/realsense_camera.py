"""Minimal RealSense color-camera wrapper."""

from __future__ import annotations

import numpy as np

from .camera import CameraFrame


class RealSenseColorCamera:
    def __init__(self, width: int, height: int, fps: int, serial: str | None = None):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("pyrealsense2 is not installed; run ./scripts/setup.sh") from exc
        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        if serial:
            config.enable_device(str(serial))
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self._config = config
        self.profile = None
        self.camera_matrix = None
        self.distortion = None

    def start(self) -> None:
        self.profile = self.pipeline.start(self._config)
        stream = self.profile.get_stream(self.rs.stream.color).as_video_stream_profile()
        intrinsics = stream.get_intrinsics()
        self.camera_matrix = np.array(
            [[intrinsics.fx, 0, intrinsics.ppx], [0, intrinsics.fy, intrinsics.ppy], [0, 0, 1]],
            dtype=np.float64,
        )
        coefficients = list(intrinsics.coeffs)
        self.distortion = np.asarray(coefficients, dtype=np.float64).reshape(-1, 1)

    def read(self) -> CameraFrame:
        if self.profile is None:
            raise RuntimeError("camera has not been started")
        frames = self.pipeline.wait_for_frames(5000)
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("RealSense returned no color frame")
        return CameraFrame(np.asanyarray(color.get_data()), color.get_timestamp() / 1000.0)

    def stop(self) -> None:
        if self.profile is not None:
            self.pipeline.stop()
            self.profile = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
