import cv2
import numpy as np

from realsense_aruco.checkerboard_pose import CheckerboardDetector


def render_checkerboard(columns, rows, square_pixels=40, border_pixels=40):
    board = np.full(((rows + 1) * square_pixels, (columns + 1) * square_pixels), 255, dtype=np.uint8)
    for row in range(rows + 1):
        for column in range(columns + 1):
            if (row + column) % 2 == 0:
                y0, x0 = row * square_pixels, column * square_pixels
                board[y0:y0 + square_pixels, x0:x0 + square_pixels] = 0
    return cv2.copyMakeBorder(
        board, border_pixels, border_pixels, border_pixels, border_pixels,
        cv2.BORDER_CONSTANT, value=255,
    )


def test_checkerboard_metric_pose():
    columns, rows = 14, 9
    image = render_checkerboard(columns, rows)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    height, width = image.shape[:2]
    camera_matrix = np.array([[800, 0, width / 2], [0, 800, height / 2], [0, 0, 1]], dtype=np.float64)
    detector = CheckerboardDetector(columns, rows, 0.02, marker_required=False)

    poses = detector.detect(image, camera_matrix, np.zeros(5))

    assert len(poses) == 1
    assert abs(poses[0].camera_T_marker[2, 3] - 0.4) < 0.01
    assert poses[0].reprojection_error_px < 0.1
