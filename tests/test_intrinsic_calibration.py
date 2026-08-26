import cv2
import numpy as np
import pytest
import yaml

from realsense_aruco.intrinsic_calibration import (
    checkerboard_detection_error_px,
    checkerboard_object_points,
    solve_intrinsics,
)
from realsense_aruco.rgb_camera import load_camera_intrinsics


def test_solve_intrinsics_synthetic_views():
    rng = np.random.default_rng(12)
    columns, rows = 9, 6
    image_size = (1280, 720)
    expected_matrix = np.array(
        [[920.0, 0.0, 641.0], [0.0, 900.0, 357.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    expected_distortion = np.array([-0.08, 0.025, 0.001, -0.0005, -0.004])
    object_points = checkerboard_object_points(columns, rows, 0.025)
    image_points = []
    for _ in range(24):
        rvec = rng.uniform([-0.45, -0.45, -0.3], [0.45, 0.45, 0.3])
        tvec = rng.uniform([-0.12, -0.08, 0.55], [0.12, 0.08, 1.0])
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, expected_matrix, expected_distortion
        )
        projected += rng.normal(0.0, 0.08, size=projected.shape).astype(np.float32)
        image_points.append(projected)

    camera_matrix, distortion, metrics = solve_intrinsics(
        image_points, image_size, columns, rows, 0.025
    )

    assert np.allclose(camera_matrix[:2, :2], expected_matrix[:2, :2], rtol=0.01, atol=2.0)
    assert np.allclose(camera_matrix[:2, 2], expected_matrix[:2, 2], atol=3.0)
    assert distortion.size == 5
    assert metrics["rms_reprojection_error_px"] < 0.2
    assert len(metrics["per_view_reprojection_error_px"]) == len(image_points)


def test_solve_intrinsics_requires_three_views():
    with pytest.raises(ValueError, match="at least 3"):
        solve_intrinsics([], (640, 480), 9, 6, 0.025)


def test_checkerboard_detection_error_for_projective_grid():
    columns, rows = 9, 6
    grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 1, 2).astype(np.float32)
    homography = np.array(
        [[42.0, 3.0, 100.0], [-2.0, 41.0, 80.0], [0.002, -0.001, 1.0]],
        dtype=np.float64,
    )
    corners = cv2.perspectiveTransform(grid, homography)

    error = checkerboard_detection_error_px(corners, columns, rows)

    assert error < 1e-3


def test_load_camera_intrinsics_checks_resolution(tmp_path):
    path = tmp_path / "intrinsics.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "image_width": 1280,
                "image_height": 720,
                "camera_matrix": [[900, 0, 640], [0, 900, 360], [0, 0, 1]],
                "distortion_coefficients": [0, 0, 0, 0, 0],
            }
        ),
        encoding="utf-8",
    )

    camera_matrix, distortion = load_camera_intrinsics(path, (1280, 720))
    assert camera_matrix.shape == (3, 3)
    assert distortion.shape == (5, 1)
    with pytest.raises(ValueError, match="recalibrate"):
        load_camera_intrinsics(path, (640, 480))
