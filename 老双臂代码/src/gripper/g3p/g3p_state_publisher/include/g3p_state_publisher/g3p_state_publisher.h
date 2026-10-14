#ifndef __G3P_STATE_PUBLISHER_H__
#define __G3P_STATE_PUBLISHER_H__

#include <ros/ros.h>

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <sys/times.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <sensor_msgs/JointState.h>

#include <g3p_msgs/Gripper.h>

#include <g3p_state_publisher/gripper3d.h>

#define DRV8825         1

#endif //__G3P_STATE_PUBLISHER_H__
