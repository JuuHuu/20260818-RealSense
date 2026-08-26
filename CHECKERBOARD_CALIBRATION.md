# 1. Marker-Aware Checkerboard Calibration

This profile uses the checkerboard shown in the supplied photograph instead of an ArUco marker. OpenCV detects the photographed board as:

```text
Internal corner columns: 14
Internal corner rows: 9
Orientation markers: required
```

The three central dots are detected by OpenCV's marker-aware sector-based checkerboard detector. They establish a consistent board orientation across robot poses.

## 2. Required square measurement

Before calibration, measure the checkerboard pitch accurately. Measure from one internal corner to the adjacent internal corner, or equivalently measure several complete squares and divide by their count.

Set the result in `config/checkerboard.yaml`:

```yaml
target:
  type: checkerboard
  columns: 14
  rows: 9
  square_size_m: 0.010
  marker_required: true
```

The current `0.010` value means 10 mm. Confirm it with a physical measurement before final calibration. A 1% square-size error causes approximately a 1% translation-scale error.

## 3. Mounting for eye-to-hand

Mount the board rigidly to the UR7e tool. Do not hold it by hand during sample capture. The board-to-tool transform may be unknown, but it must remain unchanged for every sample.

Keep the entire checkerboard visible, including its white border and three orientation dots. Avoid reflections, motion blur, board bending, and near-edge views.

## 4. Verify robot and camera

Start the UR7e driver in read-only calibration mode as described in `UR7E_ROS_CALIBRATION.md`, then check:

```bash
./scripts/ur7e_realsense.sh pose
./scripts/ur7e_realsense.sh camera-info
```

## 5. Collect checkerboard samples

Run:

```bash
./scripts/ur7e_realsense.sh collect-checkerboard-eye-to-hand
```

The camera window should draw all 126 detected internal corners and a 3D axis. Click the camera window and press `C` to capture. Both uppercase and lowercase keys are accepted.

Collect 15 to 25 stationary poses with:

1. Large rotations around multiple axes.
2. Different distances from the camera.
3. Coverage near the centre and edges of the image.
4. The complete board visible in every capture.

Samples are stored separately in `checkerboard_calibration_samples.yaml` so they cannot be mixed with ArUco samples. The final camera-to-robot transform is stored in the common `handeye_calibration.yaml`.

## 6. Solve eye-to-hand calibration

Run:

```bash
./scripts/ur7e_realsense.sh solve-checkerboard-eye-to-hand
```

The result is written to the common `handeye_calibration.yaml` as `base_T_camera`. This file has the same format whether calibration used a checkerboard or an ArUco marker.

## 7. Track the checkerboard

Run:

```bash
./scripts/ur7e_realsense.sh track-checkerboard
```

The current pose is written to `checkerboard_pose.json`. It contains:

```text
camera_T_target
base_T_target
```

The checkerboard coordinate origin is the first internal corner selected by the orientation-aware detector. Its X axis follows the 14-corner direction, its Y axis follows the 9-corner direction, and Z is normal to the board.

To use the checkerboard calibration for normal ArUco tracking, run:

```bash
./scripts/ur7e_realsense.sh track
```

The ArUco tracker reads the same `handeye_calibration.yaml` and publishes `base_T_marker`. The checkerboard does not need to be present during ArUco tracking.

## 8. Use ArUco again

The original ArUco workflow is unchanged:

```bash
./scripts/ur7e_realsense.sh collect-eye-to-hand
./scripts/ur7e_realsense.sh solve-eye-to-hand
./scripts/ur7e_realsense.sh track
```

## 9. Implemented detector details

The checkerboard version uses OpenCV's marker-aware sector-based detector. The implementation is in:

```text
src/realsense_aruco/checkerboard_pose.py
src/realsense_aruco/target_factory.py
config/checkerboard.yaml
```

For every camera frame, it:

1. Detects the 14 by 9 internal-corner pattern.
2. Requires the three-dot orientation code.
3. Localizes 126 corners with sub-pixel precision.
4. Estimates `camera_T_target` from the RealSense color intrinsics and the measured square pitch.
5. Rejects detections above the configured reprojection-error threshold.
6. Combines the target pose with `base_T_camera` after eye-to-hand calibration.

## 10. Recorded validation

The supplied checkerboard photograph was tested directly:

```text
Detected targets: 1
Detected internal corners: 126
Photograph reprojection error with approximate test intrinsics: 1.607 pixels
180-degree orientation-order maximum difference: 0.756 pixels
180-degree orientation-order mean difference: 0.156 pixels
Offline project tests: 9 passed
```

The photograph reprojection value is only a detector check because approximate intrinsics were used for that downloaded image. Live tracking uses the actual RealSense color-stream intrinsics.

## 11. Complete UR7e command sequence

Start the physical driver in a separate ROS terminal using the read-only calibration configuration described in `UR7E_ROS_CALIBRATION.md`. Then run:

```bash
cd /home/juu/Documents/20260310-isaac_for_toque/20260818-RealSense

./scripts/ur7e_realsense.sh pose
./scripts/ur7e_realsense.sh camera-info
./scripts/ur7e_realsense.sh collect-checkerboard-eye-to-hand
./scripts/ur7e_realsense.sh solve-checkerboard-eye-to-hand
./scripts/ur7e_realsense.sh track-checkerboard
```

Successful capture prints:

```text
Captured sample 1
```

The camera window must have keyboard focus. Both `c` and `C` capture, `u` and `U` undo, and `q`, `Q`, or Escape finish collection.

## 12. Final checklist

Before collecting final calibration data, confirm:

1. `square_size_m` is physically measured rather than estimated from the photograph.
2. The board is rigidly attached to the robot tool.
3. `./scripts/ur7e_realsense.sh pose` returns a current `base_T_tool0` transform.
4. All 126 corners and the coordinate axes appear in the camera window.
5. The robot is stationary before each capture.
6. At least 15 rotationally and translationally diverse samples are collected.
7. The reported calibration residuals are reviewed before using `base_T_target` for robot operations.

## 13. ROS TF stale-pose troubleshooting

The collector waits for a current `base` to `tool0` transform at every capture. It does not reuse the transform cached for the previous sample.

If capture reports a stale ROS pose, verify that the physical driver is still publishing:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states
ros2 topic hz /tf
```

Also print a current transform:

```bash
./scripts/ur7e_realsense.sh pose
```

The detailed timeout message reports the latest transform age. A continuously increasing age means the UR driver or `robot_state_publisher` stopped updating. A consistently negative age indicates that ROS and system clocks are not synchronized.
