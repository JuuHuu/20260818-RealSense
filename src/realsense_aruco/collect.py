"""Interactively collect paired robot and visual-target poses for hand-eye calibration."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

from .camera import create_color_camera
from .io_utils import load_yaml, save_yaml
from .robot_pose_provider import create_pose_provider
from .target_factory import create_target_detector, target_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/tracking.yaml")
    parser.add_argument("--robot-pose", default="robot_pose.json", help="JSON file containing base_T_gripper")
    parser.add_argument("--robot-source", choices=["file", "ros_tf"], default="file")
    parser.add_argument("--base-frame", default="base")
    parser.add_argument("--tool-frame", default="tool0")
    parser.add_argument("--output", default="calibration_samples.yaml")
    parser.add_argument("--max-error", type=float, default=2.0, help="maximum reprojection error in pixels")
    parser.add_argument("--max-robot-pose-age", type=float, default=0.25, help="maximum ROS TF age in seconds")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = load_yaml(args.config)
    camera_config = config["camera"]
    detector = create_target_detector(target_config(config))
    output_path = Path(args.output)
    data = load_yaml(output_path) if output_path.exists() else {
        "format_version": 1,
        "target": detector.metadata(),
        "samples": [],
    }
    samples = data.setdefault("samples", [])
    camera = create_color_camera(camera_config)
    robot_provider = create_pose_provider(args.robot_source, args.robot_pose, args.base_frame, args.tool_frame)
    print("Keep the robot still. Focus the camera window; press C to capture, U to undo, Q or Esc to finish.")
    with robot_provider, camera:
        while True:
            frame = camera.read()
            poses = detector.detect(frame.image, camera.camera_matrix, camera.distortion)
            pose = detector.select(poses)
            display = frame.image.copy()
            if pose:
                detector.draw(display, pose, camera.camera_matrix, camera.distortion)
            cv2.putText(display, f"samples={len(samples)}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("Hand-eye calibration capture", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("u"), ord("U")) and samples:
                samples.pop()
                save_yaml(output_path, data)
                print(f"Removed last sample; {len(samples)} remain")
            if key in (ord("c"), ord("C")):
                if pose is None:
                    print(f"{detector.target_type} target is not visible")
                    continue
                if pose.reprojection_error_px > args.max_error:
                    print(f"Rejected: reprojection error {pose.reprojection_error_px:.2f} px exceeds {args.max_error:.2f} px")
                    continue
                try:
                    robot_pose, robot_timestamp = robot_provider.get(
                        timeout_s=2.0,
                        max_age_s=(args.max_robot_pose_age if args.robot_source == "ros_tf" else None),
                    )
                except (OSError, RuntimeError, ValueError) as exc:
                    print(f"Cannot read robot pose: {exc}")
                    continue
                samples.append({
                    "captured_unix_s": time.time(),
                    "robot_timestamp": robot_timestamp,
                    "camera_timestamp_s": frame.timestamp_s,
                    "base_T_gripper": robot_pose.tolist(),
                    "camera_T_target": pose.camera_T_marker.tolist(),
                    "reprojection_error_px": pose.reprojection_error_px,
                })
                save_yaml(output_path, data)
                print(f"Captured sample {len(samples)}")
    cv2.destroyAllWindows()
    print(f"Saved {len(samples)} samples in {output_path}")


if __name__ == "__main__":
    main()
