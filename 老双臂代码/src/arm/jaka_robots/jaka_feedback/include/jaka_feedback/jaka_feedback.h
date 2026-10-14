#ifndef __JAKA_FEEDBACK_H__
#define __JAKA_FEEDBACK_H__

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
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

#include "cJSON.h"
#include "jaka_tcp.h"

#define TEST		0

#define RX_BUF_MAX	10240

#define JAKA_ON		1
#define JAKA_OFF	0



#endif //__JAKA_FEEDBACK_H__
