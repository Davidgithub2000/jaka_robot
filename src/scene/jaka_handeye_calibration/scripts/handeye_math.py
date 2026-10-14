#!/usr/bin/env python3

import math

import cv2
import numpy as np


def create_chessboard_object_points(pattern_size, square_size_m):
    cols, rows = int(pattern_size[0]), int(pattern_size[1])
    if cols <= 0 or rows <= 0:
        raise ValueError("pattern_size must contain positive inner-corner counts")
    square_size_m = float(square_size_m)
    if square_size_m <= 0.0:
        raise ValueError("square_size_m must be positive")

    points = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points[:, :2] = grid * square_size_m
    return points


def quaternion_to_matrix(qx, qy, qz, qw):
    q = np.array([float(qx), float(qy), float(qz), float(qw)], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm <= 0.0:
        raise ValueError("zero-length quaternion")
    x, y, z, w = q / norm

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation):
    m = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q


def pose_dict_to_matrix(pose):
    translation = pose.get("translation", {})
    rotation = pose.get("rotation", {})
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = [
        float(translation.get("x", 0.0)),
        float(translation.get("y", 0.0)),
        float(translation.get("z", 0.0)),
    ]

    if "matrix" in rotation:
        transform[:3, :3] = np.asarray(rotation["matrix"], dtype=np.float64).reshape(3, 3)
    else:
        transform[:3, :3] = quaternion_to_matrix(
            rotation.get("x", 0.0),
            rotation.get("y", 0.0),
            rotation.get("z", 0.0),
            rotation.get("w", 1.0),
        )
    return transform


def matrix_to_pose_dict(transform):
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    qx, qy, qz, qw = matrix_to_quaternion(matrix[:3, :3])
    return {
        "translation": {
            "x": float(matrix[0, 3]),
            "y": float(matrix[1, 3]),
            "z": float(matrix[2, 3]),
        },
        "rotation": {
            "x": float(qx),
            "y": float(qy),
            "z": float(qz),
            "w": float(qw),
        },
    }


def axis_angle_matrix(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == "y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    if axis == "z":
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    raise ValueError("unsupported Euler axis: {}".format(axis))


def euler_to_matrix(angles, order="xyz", intrinsic=False):
    order = str(order).strip().lower()
    if len(order) != 3 or any(axis not in "xyz" for axis in order):
        raise ValueError("Euler order must contain three axes from x/y/z")

    result = np.eye(3, dtype=np.float64)
    for axis, angle in zip(order, angles):
        axis_rotation = axis_angle_matrix(axis, float(angle))
        if intrinsic:
            result = result.dot(axis_rotation)
        else:
            result = axis_rotation.dot(result)
    return result


def make_transform(rotation, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def inverse_transform(transform):
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3].dot(transform[:3, 3])
    return result


def twist_stamped_to_matrix(
    msg,
    pos_unit="mm",
    rpy_unit="deg",
    rotation_type="euler",
    euler_order="xyz",
    euler_intrinsic=False,
):
    pos_scale = 0.001 if str(pos_unit).lower() in ("mm", "millimeter", "millimeters") else 1.0
    rot_scale = math.pi / 180.0 if str(rpy_unit).lower() in ("deg", "degree", "degrees") else 1.0

    translation = np.array(
        [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
        dtype=np.float64,
    ) * pos_scale

    angular = np.array(
        [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
        dtype=np.float64,
    ) * rot_scale

    if str(rotation_type).lower() == "rotvec":
        rotation, _ = cv2.Rodrigues(angular.reshape(3, 1))
    else:
        rotation = euler_to_matrix(angular, euler_order, euler_intrinsic)
    return make_transform(rotation, translation)


def solve_pnp_target_to_camera(object_points, image_points, camera_matrix, dist_coeffs):
    ok, rvec, tvec = cv2.solvePnP(
        np.asarray(object_points, dtype=np.float32),
        np.asarray(image_points, dtype=np.float32),
        np.asarray(camera_matrix, dtype=np.float64),
        np.asarray(dist_coeffs, dtype=np.float64),
    )
    if not ok:
        raise RuntimeError("solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec)
    return make_transform(rotation, tvec.reshape(3))


def calibrate_eye_in_hand(samples, method_name="TSAI"):
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    method = methods.get(str(method_name).upper())
    if method is None:
        raise ValueError("unknown handeye method: {}".format(method_name))

    r_gripper2base = []
    t_gripper2base = []
    r_target2cam = []
    t_target2cam = []
    for sample in samples:
        base_t_tool = pose_dict_to_matrix(sample["base_T_tool"])
        camera_t_target = pose_dict_to_matrix(sample["camera_T_target"])
        r_gripper2base.append(base_t_tool[:3, :3])
        t_gripper2base.append(base_t_tool[:3, 3])
        r_target2cam.append(camera_t_target[:3, :3])
        t_target2cam.append(camera_t_target[:3, 3])

    r_tool2camera, t_tool2camera = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=method,
    )
    return make_transform(r_tool2camera, np.asarray(t_tool2camera).reshape(3))
