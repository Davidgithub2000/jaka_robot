# JAKA D435i Eye-In-Hand Calibration

This package calibrates the transform from each JAKA tool frame to the D435i color optical frame.

The board in the photo is treated as GP290 12x9-20:

- Printed squares: 12 x 9
- OpenCV inner corners: 11 x 8
- Square size: 0.020 m

## Capture

Start the D435i camera publisher and JAKA driver first, then capture one arm.

If the industrial PC does not have `realsense2_camera`, use the lightweight publisher added in this workspace:

```bash
roslaunch jaka_d435i_camera left_d435i_camera.launch serial_no:=YOUR_LEFT_SERIAL
```

For one plugged-in camera you can omit `serial_no`.

Then capture the corresponding arm:

```bash
roslaunch jaka_handeye_calibration capture_handeye.launch arm:=left \
  driver_ns:=/left_arm/jaka_driver \
  image_topic:=/left_camera/color/image_raw \
  camera_info_topic:=/left_camera/color/camera_info
```

For both arms:

```bash
roslaunch jaka_d435i_camera dual_d435i_camera.launch \
  left_serial_no:=YOUR_LEFT_SERIAL \
  right_serial_no:=YOUR_RIGHT_SERIAL
roslaunch jaka_handeye_calibration capture_dual_handeye.launch
```

Move the arm to 12-20 diverse poses. Keep the board fixed in the robot base frame and visible in the camera. For each pose, type `s` in the node terminal, or publish:

```bash
rostopic pub /left_handeye/left_handeye_capture/capture std_msgs/Empty "{}" -1
rostopic pub /right_handeye/right_handeye_capture/capture std_msgs/Empty "{}" -1
```

Samples are written under:

```text
~/jaka_handeye_calibration/left/samples.yaml
~/jaka_handeye_calibration/right/samples.yaml
```

## Solve

```bash
roslaunch jaka_handeye_calibration solve_dual_handeye.launch
```

Results are written under:

```text
~/jaka_handeye_calibration/left/handeye_result.yaml
~/jaka_handeye_calibration/right/handeye_result.yaml
```

The result is `tool_T_camera`, the transform from the arm tool frame to the D435i color optical frame.

## Publish TF

```bash
roslaunch jaka_handeye_calibration publish_handeye_tf.launch
```

Default TFs:

```text
left_tool0 -> left_camera_color_optical_frame
right_tool0 -> right_camera_color_optical_frame
```

Adjust frame names in `launch/solve_handeye.launch` if your robot description uses different tool frame names.
