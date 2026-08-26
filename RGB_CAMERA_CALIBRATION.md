# 1. Ordinary RGB Camera Checkerboard Calibration

This workflow calibrates the lens intrinsics of an ordinary USB/UVC RGB camera and then uses that camera with the existing checkerboard pose and hand-eye tools. It does not require a RealSense device.

## 2. Configure the camera and checkerboard

Edit `config/rgb_checkerboard.yaml`:

```yaml
camera:
  type: opencv
  device: 0
  width: 1280
  height: 720
  fps: 30
  fourcc: MJPG
  intrinsics_file: rgb_camera_intrinsics.yaml

target:
  type: checkerboard
  columns: 14
  rows: 9
  square_size_m: 0.010
  marker_required: true
```

`columns` and `rows` are the internal-corner counts. They are one less than the printed-square counts in each direction. Set `marker_required: false` for a conventional checkerboard without orientation dots. Measure `square_size_m` from adjacent corner to adjacent corner; do not estimate it from an image.

If camera index `0` is wrong, list Linux video devices with:

```bash
v4l2-ctl --list-devices
```

Then change `device` to the correct index or device path, such as `/dev/video2`.

## 3. Calibrate the RGB camera intrinsics

Install the project and start the interactive collector:

```bash
./scripts/setup.sh
source .venv/bin/activate
rgb-checkerboard-calibrate --config config/rgb_checkerboard.yaml
```

With the UR7e convenience wrapper, the equivalent command is:

```bash
./scripts/ur7e_realsense.sh calibrate-rgb-intrinsics
```

Keep the camera resolution, focus, and zoom fixed. In the camera window:

1. Hold the board still and keep every internal corner visible.
2. Press `C` to capture a detected view.
3. Move and tilt the board to cover the image centre, edges, and corners at several distances.
4. Collect 15 to 25 varied views. The command requires at least 12 by default.
5. Press `U` to undo the last view if necessary.
6. Press `Q` or Escape to solve and save `rgb_camera_intrinsics.yaml`.

Do not capture many copies of nearly the same pose. Avoid motion blur, glare, defocus, a bent board, or views where the checkerboard occupies only a small part of the image.

The live window shows `fit error` in pixels, and every capture prints the same value. Before camera intrinsics exist, this is the RMS residual of a planar homography fitted to the detected corners. Use it to spot blurred, distorted, or poorly localized detections; it is not the final calibrated reprojection error.

After solving, the command reports the calibrated overall and worst-view reprojection errors. As an initial check, inspect the data collection if the overall error is above approximately 0.5 pixels or if the worst view is much larger than the others.

## 4. Use the RGB camera for checkerboard tracking

After the intrinsics file exists, run:

```bash
rs-aruco-track --config config/rgb_checkerboard.yaml
```

The tracker reads the calibrated camera matrix and distortion coefficients from `rgb_camera_intrinsics.yaml`. It refuses to run if the live image dimensions differ from the calibration dimensions. Recalibrate after changing resolution, focus, zoom, or any lens setting.

## 5. Use the RGB camera for robot hand-eye calibration

For a fixed camera and a checkerboard rigidly attached to the robot tool, collect paired poses with:

```bash
rs-aruco-collect \
  --config config/rgb_checkerboard.yaml \
  --output rgb_checkerboard_handeye_samples.yaml \
  --robot-source ros_tf \
  --base-frame base \
  --tool-frame tool0
```

Then solve:

```bash
rs-handeye-calibrate \
  --samples rgb_checkerboard_handeye_samples.yaml \
  --output handeye_calibration.yaml \
  --mode eye_to_hand
```

The equivalent UR7e wrapper sequence is:

```bash
./scripts/ur7e_realsense.sh collect-rgb-checkerboard-eye-to-hand
./scripts/ur7e_realsense.sh solve-rgb-checkerboard-eye-to-hand
./scripts/ur7e_realsense.sh track-rgb-checkerboard
```

The intrinsic calibration and hand-eye calibration are different:

1. Intrinsic calibration estimates the RGB camera focal lengths, optical centre, and lens distortion.
2. Hand-eye calibration estimates the rigid transform between the calibrated camera and the robot.

Complete the intrinsic calibration first.

## 6. Verify the implementation

Run the offline test suite:

```bash
./scripts/test.sh
```

Then physically validate pose accuracy at several board locations and orientations. A low checkerboard reprojection error alone does not verify the robot-to-camera transform.
