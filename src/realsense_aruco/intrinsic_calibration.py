"""Interactive checkerboard calibration for an ordinary RGB camera."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .camera import create_color_camera
from .io_utils import load_yaml, save_yaml
from .target_factory import target_config


def checkerboard_object_points(
    columns: int, rows: int, square_size_m: float
) -> np.ndarray:
    if columns < 3 or rows < 3:
        raise ValueError("checkerboard columns and rows must each be at least 3")
    if square_size_m <= 0:
        raise ValueError("checkerboard square_size_m must be positive")
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    return points * float(square_size_m)


def find_checkerboard_corners(
    image: np.ndarray, columns: int, rows: int, marker_required: bool = False
) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    if marker_required:
        flags |= cv2.CALIB_CB_MARKER
    found, corners, _ = cv2.findChessboardCornersSBWithMeta(
        gray, (columns, rows), flags
    )
    if not found:
        return None
    return np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)


def checkerboard_detection_error_px(
    corners: np.ndarray, columns: int, rows: int
) -> float:
    """Return planar homography RMS error for a detected checkerboard in pixels."""

    image_points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    expected_count = columns * rows
    if image_points.shape[0] != expected_count or not np.all(np.isfinite(image_points)):
        raise ValueError(f"detection does not contain {expected_count} finite corners")
    grid_points = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2).astype(np.float32)
    homography, _ = cv2.findHomography(grid_points, image_points, method=0)
    if homography is None or not np.all(np.isfinite(homography)):
        raise RuntimeError("could not fit a homography to the checkerboard corners")
    projected = cv2.perspectiveTransform(grid_points.reshape(-1, 1, 2), homography)
    squared_errors = np.sum(
        (projected.reshape(-1, 2) - image_points) ** 2, axis=1
    )
    return float(np.sqrt(np.mean(squared_errors)))


def solve_intrinsics(
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
    columns: int,
    rows: int,
    square_size_m: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Solve pinhole intrinsics and return matrix, distortion, and error metrics."""

    if len(image_points) < 3:
        raise ValueError("at least 3 checkerboard views are required; 12-20 diverse views are recommended")
    if image_size[0] <= 0 or image_size[1] <= 0:
        raise ValueError("image_size must contain positive width and height")
    object_template = checkerboard_object_points(columns, rows, square_size_m)
    expected_count = columns * rows
    normalized_points = []
    for index, points in enumerate(image_points):
        current = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
        if current.shape[0] != expected_count or not np.all(np.isfinite(current)):
            raise ValueError(f"view {index} does not contain {expected_count} finite corners")
        normalized_points.append(current)
    object_points = [object_template.copy() for _ in normalized_points]
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        normalized_points,
        image_size,
        None,
        None,
    )
    if not np.isfinite(rms) or not np.all(np.isfinite(camera_matrix)) or not np.all(np.isfinite(distortion)):
        raise RuntimeError("camera calibration produced non-finite values")
    per_view_errors = []
    squared_error_sum = 0.0
    point_count = 0
    for object_view, image_view, rvec, tvec in zip(
        object_points, normalized_points, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(
            object_view, rvec, tvec, camera_matrix, distortion
        )
        squared_errors = np.sum(
            (projected.reshape(-1, 2) - image_view.reshape(-1, 2)) ** 2, axis=1
        )
        per_view_errors.append(float(np.sqrt(np.mean(squared_errors))))
        squared_error_sum += float(np.sum(squared_errors))
        point_count += int(squared_errors.size)
    metrics = {
        "rms_reprojection_error_px": float(np.sqrt(squared_error_sum / point_count)),
        "opencv_rms_reprojection_error_px": float(rms),
        "mean_view_reprojection_error_px": float(np.mean(per_view_errors)),
        "max_view_reprojection_error_px": float(np.max(per_view_errors)),
        "per_view_reprojection_error_px": per_view_errors,
    }
    return camera_matrix, distortion.reshape(-1, 1), metrics


def _camera_description(config: dict[str, Any]) -> dict[str, Any]:
    camera_type = str(config.get("type", "realsense"))
    description: dict[str, Any] = {"type": camera_type}
    if camera_type == "opencv":
        description["device"] = config.get("device", 0)
    elif config.get("serial") is not None:
        description["serial"] = str(config["serial"])
    return description


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/rgb_checkerboard.yaml")
    parser.add_argument("--output", help="override camera.intrinsics_file from the configuration")
    parser.add_argument(
        "--min-samples", type=int, default=12, help="minimum accepted views before solving"
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.min_samples < 3:
        raise ValueError("--min-samples must be at least 3")
    config = load_yaml(args.config)
    camera_config = config["camera"]
    board = target_config(config)
    if board.get("type") != "checkerboard":
        raise ValueError("intrinsic calibration requires target.type: checkerboard")
    columns = int(board["columns"])
    rows = int(board["rows"])
    square_size_m = float(board["square_size_m"])
    marker_required = bool(board.get("marker_required", False))
    checkerboard_object_points(columns, rows, square_size_m)
    output_path = Path(
        args.output or camera_config.get("intrinsics_file", "rgb_camera_intrinsics.yaml")
    )
    camera = create_color_camera(camera_config, require_intrinsics=False)
    captured_points: list[np.ndarray] = []
    image_size = None
    print(
        "Show the complete checkerboard at varied positions and angles. "
        "Focus the camera window; press C to capture, U to undo, Q or Esc to solve."
    )
    try:
        with camera:
            while True:
                frame = camera.read()
                current_size = (int(frame.image.shape[1]), int(frame.image.shape[0]))
                if image_size is None:
                    image_size = current_size
                    requested = (int(camera_config["width"]), int(camera_config["height"]))
                    if image_size != requested:
                        print(
                            f"Camera requested {requested[0]}x{requested[1]} but is producing "
                            f"{image_size[0]}x{image_size[1]}; calibration will use the actual size."
                        )
                elif current_size != image_size:
                    raise RuntimeError("camera image size changed during calibration")
                corners = find_checkerboard_corners(
                    frame.image, columns, rows, marker_required
                )
                detection_error = (
                    checkerboard_detection_error_px(corners, columns, rows)
                    if corners is not None
                    else None
                )
                display = frame.image.copy()
                if corners is not None:
                    cv2.drawChessboardCorners(display, (columns, rows), corners, True)
                status = (
                    f"detected  fit error={detection_error:.3f} px"
                    if detection_error is not None
                    else "board not detected"
                )
                cv2.putText(
                    display,
                    f"samples={len(captured_points)}  {status}",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0) if corners is not None else (0, 0, 255),
                    2,
                )
                cv2.putText(
                    display,
                    "C: capture   U: undo   Q: solve",
                    (15, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                )
                cv2.imshow("RGB checkerboard intrinsic calibration", display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key in (ord("u"), ord("U")) and captured_points:
                    captured_points.pop()
                    print(f"Removed last view; {len(captured_points)} remain")
                if key in (ord("c"), ord("C")):
                    if corners is None:
                        print("Rejected: the complete checkerboard is not detected")
                    else:
                        captured_points.append(corners.copy())
                        print(
                            f"Captured view {len(captured_points)}; "
                            f"detection fit error {detection_error:.3f} px"
                        )
    finally:
        cv2.destroyAllWindows()
    if len(captured_points) < args.min_samples:
        raise SystemExit(
            f"Only {len(captured_points)} views were captured; at least {args.min_samples} are required. "
            "No calibration file was written."
        )
    assert image_size is not None
    camera_matrix, distortion, metrics = solve_intrinsics(
        captured_points, image_size, columns, rows, square_size_m
    )
    output = {
        "format_version": 1,
        "created_unix_s": time.time(),
        "camera_model": "pinhole",
        "camera_source": _camera_description(camera_config),
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "sample_count": len(captured_points),
        "checkerboard": {
            "columns": columns,
            "rows": rows,
            "square_size_m": square_size_m,
            "marker_required": marker_required,
        },
        **metrics,
    }
    save_yaml(output_path, output)
    print(f"Saved RGB camera intrinsics from {len(captured_points)} views to {output_path}")
    print(f"RMS reprojection error: {metrics['rms_reprojection_error_px']:.3f} px")
    print(f"Worst-view reprojection error: {metrics['max_view_reprojection_error_px']:.3f} px")


if __name__ == "__main__":
    main()
