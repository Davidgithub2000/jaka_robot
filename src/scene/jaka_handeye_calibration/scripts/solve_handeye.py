#!/usr/bin/env python3

import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
import yaml

from handeye_math import calibrate_eye_in_hand, matrix_to_pose_dict


def main():
    rospy.init_node("handeye_solver")
    arm_name = rospy.get_param("~arm_name", "left")
    samples_file = os.path.expandvars(os.path.expanduser(rospy.get_param("~samples_file")))
    result_file = os.path.expandvars(os.path.expanduser(rospy.get_param("~result_file")))
    parent_frame = rospy.get_param("~parent_frame", "{}_tool0".format(arm_name))
    child_frame = rospy.get_param("~child_frame", "{}_camera_color_optical_frame".format(arm_name))
    method = rospy.get_param("~method", "TSAI")
    min_samples = int(rospy.get_param("~min_samples", 12))

    with open(samples_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    samples = list(data.get("samples", []))
    if len(samples) < min_samples:
        raise RuntimeError("need at least {} samples, got {}".format(min_samples, len(samples)))

    tool_t_camera = calibrate_eye_in_hand(samples, method)
    result = {
        "arm_name": arm_name,
        "method": method,
        "sample_count": len(samples),
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "tool_T_camera": matrix_to_pose_dict(tool_t_camera),
    }

    directory = os.path.dirname(result_file)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(result_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(result, stream, default_flow_style=False, sort_keys=False)

    rospy.loginfo("[%s] wrote handeye result to %s", arm_name, result_file)
    rospy.loginfo("[%s] parent=%s child=%s", arm_name, parent_frame, child_frame)


if __name__ == "__main__":
    main()
