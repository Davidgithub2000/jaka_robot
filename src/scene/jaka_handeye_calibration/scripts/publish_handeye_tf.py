#!/usr/bin/env python3

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped

from handeye_math import pose_dict_to_matrix, matrix_to_quaternion


def main():
    rospy.init_node("publish_handeye_tf")
    result_file = os.path.expandvars(os.path.expanduser(rospy.get_param("~result_file")))
    with open(result_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    parent_frame = data["parent_frame"]
    child_frame = data["child_frame"]
    tool_t_camera = pose_dict_to_matrix(data["tool_T_camera"])
    qx, qy, qz, qw = matrix_to_quaternion(tool_t_camera[:3, :3])

    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = float(tool_t_camera[0, 3])
    msg.transform.translation.y = float(tool_t_camera[1, 3])
    msg.transform.translation.z = float(tool_t_camera[2, 3])
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(msg)
    rospy.loginfo("published static TF %s -> %s from %s", parent_frame, child_frame, result_file)
    rospy.spin()


if __name__ == "__main__":
    main()
