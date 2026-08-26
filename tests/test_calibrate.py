import cv2
import numpy as np
import pytest

from realsense_aruco.calibrate import solve
from realsense_aruco.transforms import invert, make_transform, rotation_angle_deg


def random_transform(rng, translation_scale=0.5):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = rng.uniform(-2.2, 2.2)
    rotation, _ = cv2.Rodrigues(axis * angle)
    return make_transform(rotation, rng.uniform(-translation_scale, translation_scale, size=3))


def assert_transform_close(actual, expected):
    assert np.linalg.norm(actual[:3, 3] - expected[:3, 3]) < 1e-5
    assert rotation_angle_deg(actual[:3, :3].T @ expected[:3, :3]) < 1e-3


def test_eye_in_hand_synthetic():
    rng = np.random.default_rng(4)
    gripper_T_camera = random_transform(rng, 0.12)
    base_T_target = random_transform(rng, 0.4)
    samples = []
    for _ in range(15):
        base_T_gripper = random_transform(rng)
        camera_T_target = invert(gripper_T_camera) @ invert(base_T_gripper) @ base_T_target
        samples.append({"base_T_gripper": base_T_gripper.tolist(), "camera_T_target": camera_T_target.tolist()})
    result, metrics = solve(samples, "eye_in_hand", "park")
    assert_transform_close(result, gripper_T_camera)
    assert metrics["translation_rmse_m"] < 1e-8


def test_eye_to_hand_synthetic():
    rng = np.random.default_rng(8)
    base_T_camera = random_transform(rng, 0.6)
    gripper_T_target = random_transform(rng, 0.15)
    samples = []
    for _ in range(15):
        base_T_gripper = random_transform(rng)
        camera_T_target = invert(base_T_camera) @ base_T_gripper @ gripper_T_target
        samples.append({"base_T_gripper": base_T_gripper.tolist(), "camera_T_target": camera_T_target.tolist()})
    result, metrics = solve(samples, "eye_to_hand", "park")
    assert_transform_close(result, base_T_camera)
    assert metrics["rotation_rmse_deg"] < 1e-5


def test_requires_three_samples():
    with pytest.raises(ValueError, match="at least 3"):
        solve([], "eye_in_hand", "park")

