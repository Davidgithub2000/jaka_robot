# JAKA D435i Camera

This package provides a lightweight ROS1 publisher for D435i color images and camera intrinsics using `pyrealsense2`.

It is intended for hand-eye calibration when the full `realsense2_camera` ROS driver package is not present in the workspace.

## Published Topics

Single camera default:

```text
/camera/color/image_raw
/camera/color/camera_info
```

Left camera:

```text
/left_camera/color/image_raw
/left_camera/color/camera_info
```

Right camera:

```text
/right_camera/color/image_raw
/right_camera/color/camera_info
```

## Run One Camera

```bash
roslaunch jaka_d435i_camera d435i_camera.launch show_window:=true
```

For left/right arm calibration:

```bash
roslaunch jaka_d435i_camera left_d435i_camera.launch serial_no:=YOUR_LEFT_SERIAL
roslaunch jaka_d435i_camera right_d435i_camera.launch serial_no:=YOUR_RIGHT_SERIAL
```

If only one D435i is connected, `serial_no` can be left empty.

## Check Topics

```bash
rostopic hz /camera/color/image_raw
rostopic echo /camera/color/camera_info
```

Use `rqt_image_view` if you prefer an external viewer. The launch files can also show an OpenCV window with `show_window:=true`.
