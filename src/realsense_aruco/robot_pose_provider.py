"""Robot-pose sources for files and ROS TF."""

from __future__ import annotations

from pathlib import Path
import time

import numpy as np

from .io_utils import load_pose_json
from .transforms import make_transform, quaternion_xyzw_to_rotation


class FilePoseProvider:
    def __init__(self, path: str | Path):
        self.path = path

    def get(
        self, timeout_s: float = 1.0, max_age_s: float | None = None
    ) -> tuple[np.ndarray, float | None]:
        del timeout_s, max_age_s
        return load_pose_json(self.path)

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class RosTfPoseProvider:
    """Read base_T_tool from the TF tree generated from measured joint states."""

    def __init__(self, base_frame: str = "base", tool_frame: str = "tool0"):
        try:
            import rclpy
            from rclpy.node import Node
            from tf2_ros import Buffer, TransformListener
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 Python packages are unavailable. Source /opt/ros/humble/setup.bash "
                "before activating .venv, or use scripts/ur7e_realsense.sh."
            ) from exc
        self.rclpy = rclpy
        self.base_frame = base_frame.lstrip("/")
        self.tool_frame = tool_frame.lstrip("/")
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self.node = Node("realsense_aruco_tf_reader")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node, spin_thread=False)

    def get(
        self, timeout_s: float = 1.0, max_age_s: float | None = None
    ) -> tuple[np.ndarray, float]:
        from rclpy.duration import Duration
        from rclpy.time import Time
        from tf2_ros import TransformException

        deadline = time.monotonic() + timeout_s
        last_age_s = None
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=min(0.05, timeout_s))
            try:
                stamped = self.buffer.lookup_transform(
                    self.base_frame,
                    self.tool_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                translation = stamped.transform.translation
                rotation = stamped.transform.rotation
                matrix = make_transform(
                    quaternion_xyzw_to_rotation([rotation.x, rotation.y, rotation.z, rotation.w]),
                    [translation.x, translation.y, translation.z],
                )
                timestamp = float(stamped.header.stamp.sec) + float(stamped.header.stamp.nanosec) * 1e-9
                if max_age_s is not None:
                    last_age_s = time.time() - timestamp
                    if timestamp <= 0 or last_age_s < -1.0 or last_age_s > max_age_s:
                        # The buffer may still contain the previous capture's TF.
                        # Continue spinning until a current dynamic transform arrives.
                        continue
                return matrix, timestamp
            except TransformException:
                continue
        freshness = ""
        if max_age_s is not None:
            if last_age_s is None:
                freshness = " No transform timestamp was received."
            else:
                freshness = (
                    f" Latest transform age was {last_age_s:.3f} s; "
                    f"required at most {max_age_s:.3f} s."
                )
        raise RuntimeError(
            f"No ROS TF transform {self.base_frame} -> {self.tool_frame} within {timeout_s:.1f} s. "
            "Confirm ur_robot_driver is running and /joint_states, /tf, and /tf_static are active."
            + freshness
        )

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_context and self.rclpy.ok():
            self.rclpy.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def create_pose_provider(source: str, robot_pose_file: str, base_frame: str, tool_frame: str):
    if source == "file":
        return FilePoseProvider(robot_pose_file)
    if source == "ros_tf":
        return RosTfPoseProvider(base_frame, tool_frame)
    raise ValueError(f"unknown robot pose source {source!r}")
