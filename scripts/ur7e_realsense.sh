#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/.venv/bin/activate"
cd "${PROJECT_DIR}"

usage() {
  echo "Usage: $0 {pose|collect-eye-to-hand|solve-eye-to-hand|collect-eye-in-hand|solve-eye-in-hand|track|track-eye-in-hand|collect-checkerboard-eye-to-hand|solve-checkerboard-eye-to-hand|track-checkerboard|calibrate-rgb-intrinsics|collect-rgb-checkerboard-eye-to-hand|solve-rgb-checkerboard-eye-to-hand|track-rgb-checkerboard|camera-info|test}"
}

check_tf() {
  echo "Checking live UR7e transform base -> tool0..."
  rs-ros-tf-pose --base-frame base --tool-frame tool0 --timeout 5 >/dev/null
  echo "UR7e transform is available."
}

case "${1:-}" in
  pose)
    exec rs-ros-tf-pose --base-frame base --tool-frame tool0 --timeout 5
    ;;
  collect-eye-to-hand)
    check_tf
    exec rs-aruco-collect --robot-source ros_tf --base-frame base --tool-frame tool0
    ;;
  solve-eye-to-hand)
    exec rs-handeye-calibrate --mode eye_to_hand
    ;;
  collect-eye-in-hand)
    check_tf
    exec rs-aruco-collect --robot-source ros_tf --base-frame base --tool-frame tool0
    ;;
  solve-eye-in-hand)
    exec rs-handeye-calibrate --mode eye_in_hand
    ;;
  track)
    exec rs-aruco-track
    ;;
  track-eye-in-hand)
    check_tf
    exec rs-aruco-track --robot-source ros_tf --base-frame base --tool-frame tool0
    ;;
  collect-checkerboard-eye-to-hand)
    check_tf
    exec rs-aruco-collect \
      --config config/checkerboard.yaml \
      --output checkerboard_calibration_samples.yaml \
      --robot-source ros_tf \
      --base-frame base \
      --tool-frame tool0
    ;;
  solve-checkerboard-eye-to-hand)
    exec rs-handeye-calibrate \
      --samples checkerboard_calibration_samples.yaml \
      --output handeye_calibration.yaml \
      --mode eye_to_hand
    ;;
  track-checkerboard)
    exec rs-aruco-track --config config/checkerboard.yaml
    ;;
  calibrate-rgb-intrinsics)
    exec rgb-checkerboard-calibrate --config config/rgb_checkerboard.yaml
    ;;
  collect-rgb-checkerboard-eye-to-hand)
    check_tf
    exec rs-aruco-collect \
      --config config/rgb_checkerboard.yaml \
      --output rgb_checkerboard_handeye_samples.yaml \
      --robot-source ros_tf \
      --base-frame base \
      --tool-frame tool0
    ;;
  solve-rgb-checkerboard-eye-to-hand)
    exec rs-handeye-calibrate \
      --samples rgb_checkerboard_handeye_samples.yaml \
      --output handeye_calibration.yaml \
      --mode eye_to_hand
    ;;
  track-rgb-checkerboard)
    exec rs-aruco-track --config config/rgb_checkerboard.yaml
    ;;
  camera-info)
    exec rs-camera-info
    ;;
  test)
    exec "${PROJECT_DIR}/scripts/test.sh"
    ;;
  *)
    usage
    exit 2
    ;;
esac
