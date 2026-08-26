"""Construct a calibration target detector from YAML configuration."""

from __future__ import annotations

from typing import Any

from .aruco_pose import MarkerDetector
from .checkerboard_pose import CheckerboardDetector


def create_target_detector(config: dict[str, Any]):
    target_type = config.get("type", "aruco")
    if target_type == "aruco":
        return MarkerDetector(
            config["dictionary"], float(config["size_m"]), int(config["id"])
        )
    if target_type == "checkerboard":
        return CheckerboardDetector(
            int(config["columns"]),
            int(config["rows"]),
            float(config["square_size_m"]),
            bool(config.get("marker_required", False)),
        )
    raise ValueError("target.type must be 'aruco' or 'checkerboard'")


def target_config(root_config: dict[str, Any]) -> dict[str, Any]:
    if "target" in root_config:
        return root_config["target"]
    # Backward compatibility with the original ArUco-only configuration.
    return {"type": "aruco", **root_config["marker"]}
