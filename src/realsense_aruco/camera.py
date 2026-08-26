"""Shared camera types and configuration-driven camera construction."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray
    timestamp_s: float


def create_color_camera(config: dict[str, Any], require_intrinsics: bool = True):
    """Create a RealSense or ordinary OpenCV color camera from configuration."""

    camera_type = str(config.get("type", "realsense")).lower()
    width = int(config["width"])
    height = int(config["height"])
    fps = int(config["fps"])
    if camera_type == "realsense":
        from .realsense_camera import RealSenseColorCamera

        return RealSenseColorCamera(width, height, fps, config.get("serial"))
    if camera_type == "opencv":
        from .rgb_camera import OpenCVColorCamera

        intrinsics_file = config.get("intrinsics_file") if require_intrinsics else None
        if require_intrinsics and not intrinsics_file:
            raise ValueError(
                "camera.intrinsics_file is required for an OpenCV RGB camera; "
                "run rgb-checkerboard-calibrate first"
            )
        return OpenCVColorCamera(
            device=config.get("device", 0),
            width=width,
            height=height,
            fps=fps,
            intrinsics_file=intrinsics_file,
            fourcc=config.get("fourcc"),
        )
    raise ValueError("camera.type must be 'realsense' or 'opencv'")
