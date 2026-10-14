/**
  ******************************************************************************
  * File Name          : jaka_feedback.cpp
  * Description       : jaka_feedback program body
  ******************************************************************************
  *
  * Copyright (c) 2019 HopeMotion Co., Ltd. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of HopeMotion nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for HopeMotion.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY HOPEMOTION AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL HOPEMOTION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <jaka_feedback/jaka_feedback.h>

using namespace std;


typedef struct{
	double joints[6];
	double end_pose[6];
    bool state;
} robot_t;

robot_t jaka_robot;

std::string _ipv4;

int main(int argc, char**argv)
{
	ros::init(argc, argv, "jaka_feedback");
  	ros::NodeHandle nh;
  	ros::Rate loop_rate(30);
	char rx_buf[RX_BUF_MAX];

	geometry_msgs::Quaternion quat;
	sensor_msgs::JointState joint_states;
	geometry_msgs::Pose end_pose;
	geometry_msgs::Twist endPose;
    std_msgs::Bool robot_state;
        geometry_msgs::Pose end_poserpy;

	nh.getParam("/jaka_feedback/joint_names", joint_states.name);
    nh.getParam("/jaka_feedback/joint_position", joint_states.position);
	nh.getParam("/jaka_feedback/ipv4", _ipv4);
	//nh.getParam("ipv4", _ipv4);
	ros::Publisher pub_joints = nh.advertise<sensor_msgs::JointState>("/robot_driver/joint_states", 100);
	ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose>("/robot_driver/tool_point", 100);
    ros::Publisher pub_state = nh.advertise<std_msgs::Bool>("robot_driver/robot_state", 100);
ros::Publisher pub_endpose = nh.advertise<geometry_msgs::Twist>("/robot_driver/end_pose", 100);
	
	if(tcp_init(_ipv4.c_str(), 10000))
	{
		exit(1);
	}

  	while (ros::ok())
  	{
//printf("===============\r\n");
		memset(rx_buf, 0, RX_BUF_MAX);
		if(tcp_recv(rx_buf, RX_BUF_MAX))
		{
			//printf("%s\r\n",rx_buf);
			cJSON *json = cJSON_Parse(rx_buf);
// get joints
			cJSON *joints = cJSON_GetObjectItem(json, "joint_actual_position");
			int joints_size = cJSON_GetArraySize(joints);		
			for(int i=0; i< joints_size; i++) 
    		{
        		cJSON *item = cJSON_GetArrayItem(joints, i);
        		jaka_robot.joints[i] = cJSON_GetNumberValue(item);
    		}	
// get position
			cJSON *pose = cJSON_GetObjectItem(json, "actual_position");
			int pose_size = cJSON_GetArraySize(pose);		
			for(int i=0; i< pose_size; i++) 
    		{
        		cJSON *item = cJSON_GetArrayItem(pose, i);
        		jaka_robot.end_pose[i] = cJSON_GetNumberValue(item);
				
    		}
// get states
            cJSON *state = cJSON_GetObjectItem(json, "inpos");
            if(cJSON_IsFalse(state))
            {
                jaka_robot.state = 0; 
            }
            else
            {
                jaka_robot.state = 1;
            }
//------------------------------------
			if(json)
			{
				cJSON_Delete(json);
			}
		}
//printf("---------------\r\n");
		
		joint_states.position[0] = jaka_robot.joints[0];
		joint_states.position[1] = jaka_robot.joints[1];
		joint_states.position[2] = jaka_robot.joints[2];
		joint_states.position[3] = jaka_robot.joints[3];
		joint_states.position[4] = jaka_robot.joints[4];
		joint_states.position[5] = jaka_robot.joints[5];
		pub_joints.publish(joint_states);

		end_pose.position.x = jaka_robot.end_pose[0]/1000;
		end_pose.position.y = jaka_robot.end_pose[1]/1000;
		end_pose.position.z = jaka_robot.end_pose[2]/1000;
		quat = tf::createQuaternionMsgFromRollPitchYaw(jaka_robot.end_pose[3]*M_PI/180,
														jaka_robot.end_pose[4]*M_PI/180,
														jaka_robot.end_pose[5]*M_PI/180
													  );
		end_pose.orientation.x = quat.x;
		end_pose.orientation.y = quat.y;
		end_pose.orientation.z = quat.z;
		end_pose.orientation.w = quat.w;
		pub_pose.publish(end_pose);

                end_poserpy.position.x = jaka_robot.end_pose[0];
		end_poserpy.position.y = jaka_robot.end_pose[1];
		end_poserpy.position.z = jaka_robot.end_pose[2];
		//end_poserpy.orientation.x = jaka_robot.end_pose[3]*M_PI/180;
		//end_poserpy.orientation.y = jaka_robot.end_pose[4]*M_PI/180;
		//end_poserpy.orientation.z = jaka_robot.end_pose[5]*M_PI/180;
                end_poserpy.orientation.x = jaka_robot.end_pose[3];
		end_poserpy.orientation.y = jaka_robot.end_pose[4];
		end_poserpy.orientation.z = jaka_robot.end_pose[5];
		end_poserpy.orientation.w = 0;

		endPose.linear.x = jaka_robot.end_pose[0];
		endPose.linear.y = jaka_robot.end_pose[1];
		endPose.linear.z = jaka_robot.end_pose[2];
		endPose.angular.x = jaka_robot.end_pose[3];
		endPose.angular.y = jaka_robot.end_pose[4];
		endPose.angular.z = jaka_robot.end_pose[5];
		pub_endpose.publish(endPose);

		//pub_endpose.publish(end_poserpy);

        robot_state.data = jaka_robot.state;
        pub_state.publish(robot_state);
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
	
	tcp_deinit();
  
  	return 0;
}
