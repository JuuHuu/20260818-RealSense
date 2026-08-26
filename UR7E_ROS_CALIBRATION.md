# 1. UR7e Direct ROS Calibration

This workflow reads the physical UR7e pose directly from ROS 2. It does not require manually creating or updating `robot_pose.json`.

The inspected UR7e integration uses:

```text
Measured joint topic: /joint_states
Base frame: base
Tool frame: tool0
Robot type: ur7e
Factory kinematics: config/calibration/ur7e_factory_calibration.yaml
```

The UR driver feeds `/joint_states` into `robot_state_publisher`. The RealSense calibration tool reads `base_T_tool0` from `/tf` and `/tf_static`, so forward kinematics comes from the driver URDF and factory kinematic calibration automatically.

## 2. Safety and behavior

The RealSense commands below are read-only with respect to the robot. They subscribe to TF and camera data and never publish robot commands. Move the physical robot only through the approved teach-pendant or MoveIt workflow from the UR7e integration project.

## 3. Start the physical UR7e driver

In terminal 1:

```bash
source /opt/ros/humble/setup.bash
cd /home/juu/Documents/20260310-isaac_for_toque/20260720-ROS_UR7E_intergration
python3 scripts/launch_real_ur7e.py --execute
```

Start the saved External Control program on the teach pendant. Wait until the driver reports that the robot is ready.

## 4. Verify the live robot transform

In terminal 2:

```bash
cd /home/juu/Documents/20260310-isaac_for_toque/20260818-RealSense
./scripts/ur7e_realsense.sh pose
```

This prints the current `base_T_tool0` translation, quaternion, and matrix. If it cannot find the transform, confirm that these topics exist:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo --once /joint_states
ros2 topic list | grep -E '^/(tf|tf_static)$'
```

## 5. Eye-to-hand workflow

Use eye-to-hand when the RealSense camera is fixed in the workspace and the calibration marker is rigidly attached to `tool0`.

Collect 15 to 25 varied stationary poses:

```bash
./scripts/ur7e_realsense.sh collect-eye-to-hand
```

For each pose, stop the robot, allow vibration to settle, and press `C`. Use large rotations around several axes and varied translations. Press `Q` when finished.

Solve the fixed-camera transform:

```bash
./scripts/ur7e_realsense.sh solve-eye-to-hand
```

The result is `base_T_camera` in `handeye_calibration.yaml`.

Track the marker in robot-base coordinates:

```bash
./scripts/ur7e_realsense.sh track
```

After eye-to-hand calibration, tracking does not need live robot joint information because `base_T_camera` is fixed.

## 6. Eye-in-hand workflow

Use eye-in-hand when the RealSense camera is rigidly mounted on `tool0` and the calibration marker is fixed in the workspace.

Collect samples:

```bash
./scripts/ur7e_realsense.sh collect-eye-in-hand
```

Solve:

```bash
./scripts/ur7e_realsense.sh solve-eye-in-hand
```

Track with live TF:

```bash
./scripts/ur7e_realsense.sh track-eye-in-hand
```

Eye-in-hand tracking continues to read `base_T_tool0` because the camera moves with the robot.

## 7. Direct commands

The wrapper provides UR7e defaults. The corresponding direct collection command is:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
rs-aruco-collect \
  --robot-source ros_tf \
  --base-frame base \
  --tool-frame tool0
```

Print a single transform directly:

```bash
rs-ros-tf-pose --base-frame base --tool-frame tool0
```

## 8. Frame choice

This project uses the UR controller-compatible `base` frame rather than `base_link`. The two frames in a Universal Robots ROS model can differ by a fixed orientation. Do not switch frames between calibration and tracking.

The calibration uses `tool0`. If the marker is mounted to a different custom TCP, continue using `tool0`; the hand-eye solver estimates the unknown constant marker mounting transform from multiple poses. All samples must use the same rigid mounting.

## 9. Existing JSON method

The original file method remains available for non-ROS robots:

```bash
rs-aruco-collect --robot-source file --robot-pose robot_pose.json
```

For the inspected UR7e setup, use the ROS TF wrapper instead.

## 10. Checkerboard option

The marker-aware 14 by 9 internal-corner checkerboard workflow is documented in `CHECKERBOARD_CALIBRATION.md`. It uses separate sample and target-pose files, but writes the same common `handeye_calibration.yaml` used by ArUco tracking.
