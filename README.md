# 1. RealSense ArUco Hand-Eye Calibration and Tracking

This project estimates an ArUco marker's 6D pose with an Intel RealSense color camera, calibrates the camera relative to a robot, and publishes the marker pose in both camera and robot-base coordinates.

Transforms use the convention `parent_T_child`. Translation is in metres. Quaternions are ordered `[x, y, z, w]`.

## 2. Supported camera arrangements

Choose one arrangement before collecting data:

1. `eye_in_hand`: the RealSense is rigidly mounted on the robot gripper/tool, and the calibration marker is fixed in the workspace. The result is `gripper_T_camera`.
2. `eye_to_hand`: the RealSense is fixed in the workspace, and the calibration marker is rigidly mounted on the gripper/tool. The result is `base_T_camera`.

Do not move the camera or its mounting after calibration.

## 3. Installation

On Ubuntu 22.04, run:

```bash
chmod +x scripts/setup.sh
./scripts/setup.sh
source .venv/bin/activate
rs-camera-info
```

The setup installs the stable `pyrealsense2` Python wrapper, OpenCV with ArUco support, NumPy, PyYAML, and the test tools into a local virtual environment. If the camera is connected but access is denied, install the official librealsense udev rules as described in the RealSense SDK installation documentation, reconnect the camera, and run `rs-camera-info` again.

## 4. Marker preparation

The default configuration expects marker ID 0 from `DICT_4X4_50`, with an exact black-square side length of 50 mm. Edit [config/tracking.yaml](config/tracking.yaml) to match the printed marker:

```yaml
marker:
  dictionary: DICT_4X4_50
  id: 0
  size_m: 0.050
```

Measure the black square, not the surrounding white margin. Mount the marker flat and rigidly. A larger marker generally improves range and rotation stability.

Generate the configured marker image with:

```bash
rs-aruco-generate --dictionary DICT_4X4_50 --id 0 --output aruco_marker_0.png
```

## 5. Robot pose bridge

The robot controller must continuously write the current `base_T_gripper` pose to `robot_pose.json`. Use an atomic rename when updating it so the tracker never reads a partial file. `timestamp` must be current Unix time in seconds; stale poses are rejected during eye-in-hand tracking. The format is:

```json
{
  "timestamp": 1770000000.0,
  "parent_frame": "base",
  "child_frame": "gripper",
  "matrix": [
    [1, 0, 0, 0.4],
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.3],
    [0, 0, 0, 1]
  ]
}
```

An example is provided at [examples/robot_pose.json](examples/robot_pose.json). The matrix must describe the actual flange/tool frame used during calibration. Include any configured tool-center-point offset consistently.

## 6. Collect calibration samples

Start the robot pose bridge, then run:

```bash
source .venv/bin/activate
rs-aruco-collect --robot-pose robot_pose.json
```

For each sample:

1. Move the robot to a new pose where the marker is fully visible.
2. Stop the robot and wait for vibration to settle.
3. Press `C` to capture the synchronized robot and camera poses.
4. Press `U` to remove the last sample if needed.
5. Press `Q` after collecting 15-25 samples.

Use large, varied rotations about all three axes and varied positions across the camera view. Avoid collecting many nearly identical poses. The samples are saved incrementally in `calibration_samples.yaml`.

## 7. Solve hand-eye calibration

For a camera mounted on the tool:

```bash
rs-handeye-calibrate --mode eye_in_hand
```

For a fixed camera with the marker mounted on the tool:

```bash
rs-handeye-calibrate --mode eye_to_hand
```

The result is saved to `handeye_calibration.yaml`. Inspect the reported residuals. As a practical starting point, investigate a translation RMSE above 5 mm or rotation RMSE above 1 degree. Acceptable error depends on marker size, range, robot repeatability, mounting rigidity, and the application.

## 8. Track the marker

Run:

```bash
rs-aruco-track
```

The live view shows the detected marker axes. The tracker atomically updates `marker_pose.json` with:

- `camera_T_marker` on every valid detection;
- `base_T_marker` when a valid hand-eye file is available;
- translation, quaternion, and a 4x4 matrix for each transform;
- timestamp and pixel reprojection error.

For headless use:

```bash
rs-aruco-track --no-display
```

For `eye_in_hand`, the robot pose file must continue updating while tracking. For `eye_to_hand`, the calibrated base-to-camera transform is static and the robot pose file is not required.

## 9. Verification

Run the offline tests with:

```bash
./scripts/test.sh
```

Then perform a physical validation: place the marker at a known location, compare `base_T_marker` with the robot or a measured reference, and repeat at several positions and orientations. Hand-eye residuals alone do not prove absolute accuracy.

## 10. Troubleshooting

- No device: reconnect using a USB 3 port and run `rs-camera-info`.
- Marker not found: confirm dictionary, ID, lighting, focus, and that the whole black border is visible.
- Wrong scale: correct `size_m`; pose translation scales directly with marker size.
- Pose flips or jitters: use a larger marker, improve lighting, reduce range, avoid near-edge views, and lower `smoothing_alpha` cautiously.
- Wrong robot-base pose: verify `base_T_gripper` direction and units. Do not provide `gripper_T_base`.
- Large calibration error: collect more rotationally diverse poses and check that the robot and marker were motionless at capture.

## 11. Direct UR7e ROS integration

The physical UR7e integration can provide `base_T_tool0` directly through ROS TF, with no manually maintained `robot_pose.json`:

```bash
./scripts/ur7e_realsense.sh pose
./scripts/ur7e_realsense.sh collect-eye-to-hand
./scripts/ur7e_realsense.sh solve-eye-to-hand
./scripts/ur7e_realsense.sh track
```

See [UR7E_ROS_CALIBRATION.md](UR7E_ROS_CALIBRATION.md) for the driver startup order, safety notes, eye-to-hand workflow, and eye-in-hand workflow.

## 12. Checkerboard target

The photographed marker-aware checkerboard is supported through a separate profile:

```bash
./scripts/ur7e_realsense.sh collect-checkerboard-eye-to-hand
./scripts/ur7e_realsense.sh solve-checkerboard-eye-to-hand
./scripts/ur7e_realsense.sh track
```

The checkerboard solver writes the common `handeye_calibration.yaml`, so normal ArUco tracking immediately uses that calibration. Use `track-checkerboard` only when the checkerboard itself is the tracking target. Measure and confirm `target.square_size_m` in [config/checkerboard.yaml](config/checkerboard.yaml) before collecting final samples. See [CHECKERBOARD_CALIBRATION.md](CHECKERBOARD_CALIBRATION.md) for the complete procedure.

## 13. Ordinary RGB camera

An ordinary USB/UVC RGB camera is supported through OpenCV. Unlike RealSense, it does not provide factory intrinsics, so calibrate it first:

```bash
source .venv/bin/activate
rgb-checkerboard-calibrate --config config/rgb_checkerboard.yaml
rs-aruco-track --config config/rgb_checkerboard.yaml
```

The same RGB camera profile can be passed to `rs-aruco-collect` for robot hand-eye calibration. See [RGB_CAMERA_CALIBRATION.md](RGB_CAMERA_CALIBRATION.md) for configuration, capture guidance, and the complete command sequence.
