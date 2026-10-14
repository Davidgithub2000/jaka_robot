#ifndef __g3p_DRIVER_H__
#define __g3p_DRIVER_H__

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

#include <serial/serial.h> 
#include <sensor_msgs/JointState.h>

#include <g3p_msgs/Gripper.h>

#define DRV8825         1

#if DRV8825
#define TXD_MAX         8
#else
#define TXD_MAX         12
#define STEP_MAX        11                          // 夹爪最大行程11CM
#define MOTOR_1_8_STEP        200                   // 1.8度步进 200 脉冲为一圈
#define DRIVER_SUBDIVIDE      16                    // 驱动为16细分
#endif

#endif //__ROCR6_DRIVER_H__
