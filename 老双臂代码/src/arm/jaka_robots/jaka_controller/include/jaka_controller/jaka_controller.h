#ifndef __JAKA_CONTROLLER_H__
#define __JAKA_CONTROLLER_H__

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/Pose.h>
#include <jaka_msgs/Move.h>

#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

#include "cJSON.h"
#include "jaka_tcp.h"

#define RX_BUF_MAX			256
#define JOINT_MAX			6

#define JAKA_ON				1
#define JAKA_OFF			0

#define ROBOT_INIT			0
#define ROBOT_IDLE			1
#define ROBOT_EXEC			2
#define ROBOT_QUIT			3

#define JAKA_MSG_NULL		0
#define JAKA_MSG_POWER_ON	1
#define JAKA_MSG_POWER_OFF	2
#define JAKA_MSG_ROBOT_ENB	3
#define JAKA_MSG_ROBOT_DIS	4
#define JAKA_MSG_ROBOT_QUIT	5
#define JAKA_MSG_MOVE_J		6
#define JAKA_MSG_MOVE_L		7
#define JAKA_MSG_MOVE_E		8
#define JAKA_MSG_PLAY		9
#define JAKA_MSG_PAUSE		10
#define JAKA_MSG_STOP		11	
#define JAKA_MSG_SET_DIO    12

//#define __DEBUG__  
#ifdef __DEBUG__  
#define DEBUG(format,...) printf(format, ##__VA_ARGS__)  
#else  
#define DEBUG(format,...)  
#endif


#endif //__JAKA_CONTROLLER_H__
