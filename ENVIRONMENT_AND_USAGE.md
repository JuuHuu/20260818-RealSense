# 1. Virtual Environment and Usage Guide

This document records the Python environment and the different methods for using the RealSense ArUco calibration and tracking tools.

All transforms use `parent_T_child` notation. Translation values are in metres, and quaternions use `[x, y, z, w]` order.

## 2. Installed virtual environment

The project uses this local environment:

```text
/home/juu/Documents/20260310-isaac_for_toque/20260818-RealSense/.venv
```

The recorded package versions are:

```text
Ubuntu: 22.04
Python: 3.10.12
NumPy: 2.2.6
OpenCV contrib: 4.14.0.94
pyrealsense2: 2.58.3.10794
PyYAML: 6.0.3
pytest: 8.4.2
```

Because `python3-venv` is unavailable on this computer, `scripts/setup.sh` automatically bootstraps `virtualenv` into `.venv-bootstrap` and uses it to create `.venv`.

The `orca-core` and `generate-parameter-library-py` messages from the first installation were warnings about packages visible in the system or ROS environment. The RealSense installation completed successfully. The setup script now removes the external `PYTHONPATH` during installation so ROS packages do not affect dependency resolution.

## 3. Create or recreate the environment

From the project directory, run:

```bash
./scripts/setup.sh
```

The script creates or refreshes `.venv` and installs the project in editable mode.

Installing Ubuntu's standard environment support is optional because the fallback works. To install it:

```bash
sudo apt update
sudo apt install python3.10-venv
```

## 4. Activate and leave the environment

Activate the environment in each new terminal:

```bash
cd ~/Documents/20260310-isaac_for_toque/20260818-RealSense
source .venv/bin/activate
```

Verify it:

```bash
which python
python --version
```

The expected interpreter is:

```text
/home/juu/Documents/20260310-isaac_for_toque/20260818-RealSense/.venv/bin/python
```

Leave the environment with:

```bash
deactivate
```

If ROS was sourced in the same terminal and causes import conflicts, use a clean terminal or run:

```bash
unset PYTHONPATH
source .venv/bin/activate
```

The robot pose bridge can run in a separate ROS-enabled terminal while this tracker runs in the isolated terminal.

## 5. Verify the installation

Run the offline tests:

```bash
./scripts/test.sh
```

The expected result is `9 passed`.

Check for connected cameras:

```bash
rs-camera-info
```

If this reports a udev or USB permission error, install the official RealSense udev rules and reconnect the camera. If it reports no camera, check the USB 3 cable and port.

## 6. Configure the camera and marker

Edit `config/tracking.yaml` before calibration:

```yaml
camera:
  serial: null
  width: 1280
  height: 720
  fps: 30

marker:
  dictionary: DICT_4X4_50
  id: 0
  size_m: 0.050
```

Set `serial` when multiple RealSense cameras are connected. Set `size_m` to the measured side length of the black marker square, excluding the white margin.

Generate the default marker image:

```bash
rs-aruco-generate \
  --dictionary DICT_4X4_50 \
  --id 0 \
  --output aruco_marker_0.png
```

## 7. Robot pose input

The robot controller or a bridge program must update `robot_pose.json` with `base_T_gripper`:

```json
{
  "timestamp": 1770000000.0,
  "parent_frame": "base",
  "child_frame": "gripper",
  "matrix": [
    [1.0, 0.0, 0.0, 0.4],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.3],
    [0.0, 0.0, 0.0, 1.0]
  ]
}
```

The timestamp must be Unix time in seconds, and translation must be in metres. Update the file with a temporary file followed by an atomic rename so the tracker cannot read partial data.

## 8. Method A: camera-frame tracking only

This method does not require robot calibration. Ensure `handeye_calibration.yaml` does not exist, then run:

```bash
rs-aruco-track
```

The output `marker_pose.json` contains `camera_T_marker`.

For headless operation:

```bash
rs-aruco-track --no-display
```

## 9. Method B: eye-in-hand calibration

Use this method when the RealSense is rigidly mounted on the robot tool. Fix the calibration marker in the workspace.

Start the robot pose bridge and collect samples:

```bash
rs-aruco-collect --robot-pose robot_pose.json
```

Stop the robot at each pose and press `C`. Collect 15 to 25 poses with varied positions and rotations about all three axes. Press `U` to remove the last sample and `Q` to finish.

Solve the calibration:

```bash
rs-handeye-calibrate --mode eye_in_hand
```

The result is `gripper_T_camera`. During tracking, the robot pose must continue updating:

```bash
rs-aruco-track
```

The calculation is:

```text
base_T_marker = base_T_gripper * gripper_T_camera * camera_T_marker
```

## 10. Method C: eye-to-hand calibration

Use this method when the RealSense is fixed outside the robot. Rigidly mount the calibration marker on the robot tool.

Collect samples:

```bash
rs-aruco-collect --robot-pose robot_pose.json
```

After collecting 15 to 25 varied stationary poses, solve:

```bash
rs-handeye-calibrate --mode eye_to_hand
```

The result is `base_T_camera`. Start tracking with:

```bash
rs-aruco-track
```

The calculation is:

```text
base_T_marker = base_T_camera * camera_T_marker
```

The robot pose file is not required during eye-to-hand tracking after calibration.

## 11. Alternative calibration methods

The default solver is Park. Other available OpenCV methods are:

```bash
rs-handeye-calibrate --mode eye_in_hand --method tsai
rs-handeye-calibrate --mode eye_in_hand --method park
rs-handeye-calibrate --mode eye_in_hand --method horaud
rs-handeye-calibrate --mode eye_in_hand --method andreff
rs-handeye-calibrate --mode eye_in_hand --method daniilidis
```

Use `eye_to_hand` instead when the camera is fixed. Compare residuals and physically validate the result at several marker positions.

## 12. Runtime files

```text
calibration_samples.yaml   Paired robot and camera observations
handeye_calibration.yaml   Solved camera-to-robot calibration
marker_pose.json           Latest marker detection and 6D pose
robot_pose.json            Live pose written by the robot bridge
```

When the marker is visible, `marker_pose.json` contains `detected: true`. When it is lost, the file contains `detected: false`; downstream programs must not reuse an old pose.

## 13. Typical command sequences

Eye-in-hand:

```bash
source .venv/bin/activate
rs-camera-info
rs-aruco-collect --robot-pose robot_pose.json
rs-handeye-calibrate --mode eye_in_hand
rs-aruco-track
```

Eye-to-hand:

```bash
source .venv/bin/activate
rs-camera-info
rs-aruco-collect --robot-pose robot_pose.json
rs-handeye-calibrate --mode eye_to_hand
rs-aruco-track
```

Camera-only headless tracking:

```bash
source .venv/bin/activate
rs-aruco-track --no-display
```

## 14. UR7e direct ROS method

For the physical UR7e integration, no manually updated `robot_pose.json` is required. Use the ready commands documented in `UR7E_ROS_CALIBRATION.md`:

```bash
./scripts/ur7e_realsense.sh pose
./scripts/ur7e_realsense.sh collect-eye-to-hand
./scripts/ur7e_realsense.sh solve-eye-to-hand
./scripts/ur7e_realsense.sh track
```
