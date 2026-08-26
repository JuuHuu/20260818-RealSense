"""Marker-aware checkerboard detection and metric pose estimation."""

from __future__ import annotations

import cv2
import numpy as np

from .aruco_pose import MarkerPose
from .transforms import rvec_tvec_to_transform


class CheckerboardDetector:
    def __init__(self, columns: int, rows: int, square_size_m: float, marker_required: bool = True):
        if columns < 3 or rows < 3:
            raise ValueError("checkerboard columns and rows must each be at least 3")
        if square_size_m <= 0:
            raise ValueError("checkerboard square_size_m must be positive")
        self.columns = int(columns)
        self.rows = int(rows)
        self.square_size_m = float(square_size_m)
        self.marker_required = bool(marker_required)
        self.target_id = None
        self.target_type = "checkerboard"
        self.frame_name = "checkerboard"
        self.axis_length_m = square_size_m * 3.0
        grid = np.zeros((self.rows * self.columns, 3), dtype=np.float64)
        grid[:, :2] = np.mgrid[0:self.columns, 0:self.rows].T.reshape(-1, 2)
        self.object_points = grid * self.square_size_m

    def detect(self, image: np.ndarray, camera_matrix: np.ndarray, distortion: np.ndarray) -> list[MarkerPose]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        if self.marker_required:
            flags |= cv2.CALIB_CB_MARKER
        found, corners, _ = cv2.findChessboardCornersSBWithMeta(
            gray, (self.columns, self.rows), flags
        )
        if not found:
            return []
        image_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_IPPE,
        )
        if not success or float(tvec.reshape(3)[2]) <= 0:
            return []
        rvec, tvec = cv2.solvePnPRefineLM(
            self.object_points, image_points, camera_matrix, distortion, rvec, tvec
        )
        projected, _ = cv2.projectPoints(
            self.object_points, rvec, tvec, camera_matrix, distortion
        )
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
        return [MarkerPose(-1, image_points, rvec_tvec_to_transform(rvec, tvec), rvec, tvec, error)]

    def select(self, poses: list[MarkerPose]) -> MarkerPose | None:
        return poses[0] if poses else None

    def draw(self, image: np.ndarray, pose: MarkerPose, camera_matrix: np.ndarray, distortion: np.ndarray) -> None:
        corners = pose.corners.reshape(-1, 1, 2).astype(np.float32)
        cv2.drawChessboardCorners(image, (self.columns, self.rows), corners, True)
        cv2.drawFrameAxes(
            image, camera_matrix, distortion, pose.rvec, pose.tvec, self.axis_length_m, 2
        )
        origin = tuple(pose.corners[0].astype(int))
        cv2.putText(
            image,
            f"checkerboard err={pose.reprojection_error_px:.2f}px",
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
        )

    def metadata(self) -> dict:
        return {
            "type": self.target_type,
            "columns": self.columns,
            "rows": self.rows,
            "square_size_m": self.square_size_m,
            "marker_required": self.marker_required,
        }
