"""Track an ArUco or checkerboard target and publish its 6D pose as JSON."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import time
from pathlib import Path

import cv2
import numpy as np

from .camera import create_color_camera
from .io_utils import atomic_write_json, load_yaml
from .robot_pose_provider import create_pose_provider
from .target_factory import create_target_detector, target_config
from .transforms import make_transform, transform_dict, validate_transform


def _smooth(previous: np.ndarray | None, current: np.ndarray, alpha: float) -> np.ndarray:
    if previous is None or alpha >= 1:
        return current.copy()
    result = np.eye(4, dtype=np.float64)
    result[:3, 3] = (1 - alpha) * previous[:3, 3] + alpha * current[:3, 3]
    delta = previous[:3, :3].T @ current[:3, :3]
    rvec, _ = cv2.Rodrigues(delta)
    step, _ = cv2.Rodrigues(alpha * rvec)
    result[:3, :3] = previous[:3, :3] @ step
    return result


def _base_pose(handeye: dict, camera_T_marker: np.ndarray, robot_provider, max_robot_pose_age_s: float) -> np.ndarray:
    mode = handeye.get("mode")
    if mode == "eye_to_hand":
        return validate_transform(handeye["base_T_camera"], "base_T_camera") @ camera_T_marker
    if mode == "eye_in_hand":
        base_T_gripper, robot_timestamp = robot_provider.get(
            timeout_s=0.1, max_age_s=max_robot_pose_age_s
        )
        if robot_timestamp is None:
            raise ValueError("robot pose has no Unix 'timestamp'")
        age = time.time() - float(robot_timestamp)
        if age < -1.0 or age > max_robot_pose_age_s:
            raise ValueError(f"robot pose is stale or unsynchronized (age {age:.3f} s)")
        gripper_T_camera = validate_transform(handeye["gripper_T_camera"], "gripper_T_camera")
        return base_T_gripper @ gripper_T_camera @ camera_T_marker
    raise ValueError("hand-eye file has invalid mode")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/tracking.yaml")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--robot-source", choices=["file", "ros_tf"], default="file")
    parser.add_argument("--base-frame", default="base")
    parser.add_argument("--tool-frame", default="tool0")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = load_yaml(args.config)
    camera_config, tracking = config["camera"], config["tracking"]
    detector = create_target_detector(target_config(config))
    handeye_path = Path(tracking["handeye_file"])
    handeye = load_yaml(handeye_path) if handeye_path.exists() else None
    robot_provider = None
    if handeye and handeye.get("mode") == "eye_in_hand":
        robot_provider = create_pose_provider(
            args.robot_source, tracking["robot_pose_file"], args.base_frame, args.tool_frame
        )
    max_error = float(tracking.get("max_reprojection_error_px", 2.0))
    max_robot_pose_age_s = float(tracking.get("max_robot_pose_age_s", 0.25))
    alpha = float(tracking.get("smoothing_alpha", 1.0))
    if not 0 < alpha <= 1:
        raise ValueError("smoothing_alpha must be in (0, 1]")
    show = bool(tracking.get("show_window", True)) and not args.no_display
    filtered = None
    camera = create_color_camera(camera_config)
    print(f"Tracking {detector.frame_name}. Press Q or Esc to stop.")
    provider_context = robot_provider if robot_provider is not None else nullcontext()
    with provider_context, camera:
        while True:
            frame = camera.read()
            poses = detector.detect(frame.image, camera.camera_matrix, camera.distortion)
            pose = detector.select([item for item in poses if item.reprojection_error_px <= max_error])
            if pose:
                filtered = _smooth(filtered, pose.camera_T_marker, alpha)
                base_T_target = None
                output = {
                    "timestamp_unix_s": time.time(),
                    "camera_timestamp_s": frame.timestamp_s,
                    "detected": True,
                    "target_type": detector.target_type,
                    "reprojection_error_px": pose.reprojection_error_px,
                }
                camera_key = "camera_T_marker" if detector.target_type == "aruco" else "camera_T_target"
                output[camera_key] = transform_dict(filtered, "camera", detector.frame_name)
                if detector.target_id is not None:
                    output["marker_id"] = detector.target_id
                if handeye:
                    try:
                        base_T_target = _base_pose(
                            handeye, filtered, robot_provider, max_robot_pose_age_s
                        )
                        base_key = "base_T_marker" if detector.target_type == "aruco" else "base_T_target"
                        output[base_key] = transform_dict(
                            base_T_target, "base", detector.frame_name
                        )
                    except (OSError, RuntimeError, ValueError) as exc:
                        output["base_pose_error"] = str(exc)
                atomic_write_json(tracking["output_file"], output)
                status = "camera xyz [m]: " + " ".join(
                    f"{value:+.4f}" for value in filtered[:3, 3]
                )
                if base_T_target is not None:
                    status += " | robot/base xyz [m]: " + " ".join(
                        f"{value:+.4f}" for value in base_T_target[:3, 3]
                    )
                print("\r" + status, end="", flush=True)
            else:
                lost_output = {
                    "timestamp_unix_s": time.time(),
                    "camera_timestamp_s": frame.timestamp_s,
                    "detected": False,
                    "target_type": detector.target_type,
                }
                if detector.target_id is not None:
                    lost_output["marker_id"] = detector.target_id
                atomic_write_json(tracking["output_file"], lost_output)
            if show:
                display = frame.image.copy()
                if pose:
                    detector.draw(display, pose, camera.camera_matrix, camera.distortion)
                cv2.imshow("Camera target tracking", display)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    break
    if show:
        cv2.destroyAllWindows()
    print()


if __name__ == "__main__":
    main()
