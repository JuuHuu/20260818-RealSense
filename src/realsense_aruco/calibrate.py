"""Compute an eye-in-hand or eye-to-hand calibration from paired samples."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from .io_utils import load_yaml, save_yaml
from .transforms import invert, make_transform, rotation_angle_deg, transform_dict, validate_transform


METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    accumulator = np.sum(rotations, axis=0)
    u, _, vt = np.linalg.svd(accumulator)
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def _residuals(mode: str, robot_poses: list[np.ndarray], camera_poses: list[np.ndarray], calibration: np.ndarray) -> dict:
    constants = []
    for base_T_gripper, camera_T_target in zip(robot_poses, camera_poses):
        if mode == "eye_in_hand":
            constants.append(base_T_gripper @ calibration @ camera_T_target)
        else:
            constants.append(invert(base_T_gripper) @ calibration @ camera_T_target)
    mean_translation = np.mean([item[:3, 3] for item in constants], axis=0)
    mean_rotation = _mean_rotation([item[:3, :3] for item in constants])
    translation_errors = [float(np.linalg.norm(item[:3, 3] - mean_translation)) for item in constants]
    rotation_errors = [rotation_angle_deg(mean_rotation.T @ item[:3, :3]) for item in constants]
    return {
        "translation_rmse_m": float(np.sqrt(np.mean(np.square(translation_errors)))),
        "translation_max_m": max(translation_errors),
        "rotation_rmse_deg": float(np.sqrt(np.mean(np.square(rotation_errors)))),
        "rotation_max_deg": max(rotation_errors),
    }


def solve(samples: list[dict], mode: str, method: str) -> tuple[np.ndarray, dict]:
    if mode not in {"eye_in_hand", "eye_to_hand"}:
        raise ValueError("mode must be eye_in_hand or eye_to_hand")
    if method not in METHODS:
        raise ValueError(f"unknown method {method}")
    if len(samples) < 3:
        raise ValueError("at least 3 samples are required; 10-20 diverse poses are recommended")

    robot_poses = [validate_transform(item["base_T_gripper"], f"sample {index} base_T_gripper") for index, item in enumerate(samples)]
    camera_poses = [validate_transform(item["camera_T_target"], f"sample {index} camera_T_target") for index, item in enumerate(samples)]
    handeye_robot_poses = robot_poses if mode == "eye_in_hand" else [invert(item) for item in robot_poses]
    rotation, translation = cv2.calibrateHandEye(
        [item[:3, :3] for item in handeye_robot_poses],
        [item[:3, 3] for item in handeye_robot_poses],
        [item[:3, :3] for item in camera_poses],
        [item[:3, 3] for item in camera_poses],
        method=METHODS[method],
    )
    calibration = make_transform(rotation, translation)
    return calibration, _residuals(mode, robot_poses, camera_poses, calibration)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", default="calibration_samples.yaml")
    parser.add_argument("--output", default="handeye_calibration.yaml")
    parser.add_argument("--mode", choices=["eye_in_hand", "eye_to_hand"], required=True)
    parser.add_argument("--method", choices=sorted(METHODS), default="park")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    source = load_yaml(args.samples)
    samples = source.get("samples")
    if not isinstance(samples, list):
        raise ValueError(f"{args.samples} has no samples list")
    result, metrics = solve(samples, args.mode, args.method)
    if args.mode == "eye_in_hand":
        key, parent, child = "gripper_T_camera", "gripper", "camera"
    else:
        key, parent, child = "base_T_camera", "base", "camera"
    output = {
        "format_version": 1,
        "created_unix_s": time.time(),
        "mode": args.mode,
        "method": args.method,
        "sample_count": len(samples),
        key: result.tolist(),
        "transform": transform_dict(result, parent, child),
        "residuals": metrics,
    }
    calibration_target = source.get("target", source.get("marker"))
    if calibration_target is not None:
        output["calibration_target"] = calibration_target
    save_yaml(args.output, output)
    print(f"Saved {args.mode} calibration from {len(samples)} samples to {Path(args.output)}")
    print(f"Translation RMSE: {metrics['translation_rmse_m'] * 1000:.2f} mm")
    print(f"Rotation RMSE: {metrics['rotation_rmse_deg']:.3f} deg")


if __name__ == "__main__":
    main()
