#!/usr/bin/env python3

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import numpy as np
import rospy
import yaml

from handeye_math import inverse_transform, pose_dict_to_matrix


def main():
    rospy.init_node("verify_handeye")
    samples_file = os.path.expandvars(os.path.expanduser(rospy.get_param("~samples_file")))
    result_file = os.path.expandvars(os.path.expanduser(rospy.get_param("~result_file")))

    with open(samples_file, "r", encoding="utf-8") as stream:
        samples_data = yaml.safe_load(stream) or {}
    with open(result_file, "r", encoding="utf-8") as stream:
        result_data = yaml.safe_load(stream) or {}

    tool_t_camera = pose_dict_to_matrix(result_data["tool_T_camera"])
    base_t_targets = []
    for sample in samples_data.get("samples", []):
        base_t_tool = pose_dict_to_matrix(sample["base_T_tool"])
        camera_t_target = pose_dict_to_matrix(sample["camera_T_target"])
        base_t_targets.append(base_t_tool.dot(tool_t_camera).dot(camera_t_target))

    if len(base_t_targets) < 2:
        raise RuntimeError("need at least two samples to verify")

    translations = np.array([item[:3, 3] for item in base_t_targets], dtype=np.float64)
    mean = translations.mean(axis=0)
    errors = np.linalg.norm(translations - mean, axis=1)
    rospy.loginfo("target translation mean in base: %s", mean.tolist())
    rospy.loginfo("target translation residual mean: %.6f m", float(errors.mean()))
    rospy.loginfo("target translation residual max: %.6f m", float(errors.max()))

    first_inv = inverse_transform(base_t_targets[0])
    rot_errors = []
    for transform in base_t_targets[1:]:
        delta = first_inv.dot(transform)
        trace_value = max(-1.0, min(1.0, (np.trace(delta[:3, :3]) - 1.0) / 2.0))
        rot_errors.append(np.degrees(np.arccos(trace_value)))
    rospy.loginfo("target rotation residual max: %.6f deg", float(max(rot_errors) if rot_errors else 0.0))


if __name__ == "__main__":
    main()
