#!/usr/bin/env python3


def parse_camera_config(config, camera_name):
    cameras = dict((config or {}).get("cameras", {}))
    if camera_name not in cameras:
        raise KeyError("camera '{}' not found in config".format(camera_name))
    entry = dict(cameras[camera_name] or {})
    entry.setdefault("serial_no", "")
    entry.setdefault("namespace", "/camera")
    entry.setdefault("frame_id", "camera_color_optical_frame")
    entry.setdefault("width", 1280)
    entry.setdefault("height", 720)
    entry.setdefault("fps", 30)
    return entry


def camera_info_dict_from_intrinsics(intrinsics, frame_id):
    fx = float(intrinsics.fx)
    fy = float(intrinsics.fy)
    cx = float(intrinsics.ppx)
    cy = float(intrinsics.ppy)
    width = int(intrinsics.width)
    height = int(intrinsics.height)
    coeffs = [float(value) for value in list(getattr(intrinsics, "coeffs", []))]

    return {
        "frame_id": str(frame_id),
        "width": width,
        "height": height,
        "distortion_model": "plumb_bob",
        "d": coeffs,
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }
