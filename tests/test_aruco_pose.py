import cv2
import numpy as np

from realsense_aruco.aruco_pose import MarkerDetector, dictionary_from_name


def test_detects_rendered_marker_with_metric_pose():
    dictionary = dictionary_from_name("DICT_4X4_50")
    marker = cv2.aruco.generateImageMarker(dictionary, 7, 200)
    image = np.full((400, 400, 3), 255, dtype=np.uint8)
    image[100:300, 100:300] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    camera_matrix = np.array([[600, 0, 200], [0, 600, 200], [0, 0, 1]], dtype=np.float64)

    poses = MarkerDetector("DICT_4X4_50", 0.05).detect(image, camera_matrix, np.zeros(5))

    assert len(poses) == 1
    assert poses[0].marker_id == 7
    assert abs(poses[0].camera_T_marker[2, 3] - 0.15) < 0.005
    assert poses[0].reprojection_error_px < 0.1
