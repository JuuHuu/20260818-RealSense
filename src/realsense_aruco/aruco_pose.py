"""ArUco detection and square-marker pose estimation."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .transforms import rvec_tvec_to_transform


@dataclass(frozen=True)
class MarkerPose:
    marker_id: int
    corners: np.ndarray
    camera_T_marker: np.ndarray
    rvec: np.ndarray
    tvec: np.ndarray
    reprojection_error_px: float


def dictionary_from_name(name: str):
    if not hasattr(cv2.aruco, name):
        valid = sorted(item for item in dir(cv2.aruco) if item.startswith("DICT_"))
        raise ValueError(f"Unknown ArUco dictionary {name!r}; valid values: {', '.join(valid)}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


class MarkerDetector:
    def __init__(self, dictionary_name: str, marker_size_m: float, marker_id: int | None = None):
        if marker_size_m <= 0:
            raise ValueError("marker_size_m must be positive")
        self.dictionary = dictionary_from_name(dictionary_name)
        self.dictionary_name = dictionary_name
        self.marker_size_m = marker_size_m
        self.target_id = marker_id
        self.target_type = "aruco"
        self.frame_name = f"aruco_{marker_id}" if marker_id is not None else "aruco"
        self.axis_length_m = marker_size_m * 0.6
        parameters = cv2.aruco.DetectorParameters_create() if not hasattr(cv2.aruco, "DetectorParameters") else cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self.dictionary, parameters) if hasattr(cv2.aruco, "ArucoDetector") else None
        self._parameters = parameters

    def detect(self, image: np.ndarray, camera_matrix: np.ndarray, distortion: np.ndarray) -> list[MarkerPose]:
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self._parameters)
        if ids is None:
            return []

        half = self.marker_size_m / 2.0
        object_points = np.array(
            [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
            dtype=np.float64,
        )
        results = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            image_points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success or float(tvec.reshape(3)[2]) <= 0:
                continue
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points) ** 2, axis=1))))
            results.append(MarkerPose(int(marker_id), image_points, rvec_tvec_to_transform(rvec, tvec), rvec, tvec, error))
        return results

    def select(self, poses: list[MarkerPose]) -> MarkerPose | None:
        if self.target_id is None:
            return poses[0] if poses else None
        return next((pose for pose in poses if pose.marker_id == self.target_id), None)

    def draw(self, image: np.ndarray, pose: MarkerPose, camera_matrix: np.ndarray, distortion: np.ndarray) -> None:
        draw_pose(image, pose, camera_matrix, distortion, self.axis_length_m)

    def metadata(self) -> dict:
        return {
            "type": self.target_type,
            "dictionary": self.dictionary_name,
            "id": self.target_id,
            "size_m": self.marker_size_m,
        }


def draw_pose(image: np.ndarray, pose: MarkerPose, camera_matrix: np.ndarray, distortion: np.ndarray, axis_length_m: float) -> None:
    cv2.polylines(image, [pose.corners.astype(np.int32)], True, (0, 255, 0), 2)
    cv2.drawFrameAxes(image, camera_matrix, distortion, pose.rvec, pose.tvec, axis_length_m, 2)
    origin = tuple(pose.corners[0].astype(int))
    cv2.putText(image, f"id={pose.marker_id} err={pose.reprojection_error_px:.2f}px", origin, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
