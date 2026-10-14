/**
  ******************************************************************************
  * File Name          : jaka_controller.cpp
  * Description       : jaka_controller program body
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
#include <jaka_controller/jaka_controller.h>

using namespace std;

/* X Y Z roll pitch yaw */
typedef struct{
	double data[6];
}pose_t;

typedef struct{
	double position;
	double velocity;
	double torque;
}motor_t;

typedef struct{
	int msgId;
	int errorCode;
}resp_t;

typedef struct{
	double joints[6] = {0, 0, 0, 0, 0, 0};
	double end_pose[6] = {0, 0, 0, 0, 0, 0};
	unsigned char state = 0;
	resp_t resp;
}robot_t;

robot_t jaka;

double _default_speed;
double _default_accel;
std::string _ipv4;

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_set_tio_do1(unsigned char status)
{
	cJSON *root=cJSON_CreateObject(); 
	cJSON_AddStringToObject(root,"cmdName","set_digital_output"); 
	cJSON_AddNumberToObject(root,"type",1);
	cJSON_AddNumberToObject(root,"index",0);
	if(status)
	{
		cJSON_AddNumberToObject(root,"value",1);
	}
	else
	{
		cJSON_AddNumberToObject(root,"value",0);
	}
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_set_tio_do2(unsigned char status)
{
	cJSON *root=cJSON_CreateObject(); 
	cJSON_AddStringToObject(root,"cmdName","set_digital_output"); 
	cJSON_AddNumberToObject(root,"type",1);
	cJSON_AddNumberToObject(root,"index",1);
	if(status)
	{
		cJSON_AddNumberToObject(root,"value",1);
	}
	else
	{
		cJSON_AddNumberToObject(root,"value",0);
	}
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
void init_tio_do()
{
	jaka_set_tio_do1(0);
	jaka_set_tio_do2(0);
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_set_power(unsigned char status)
{
	cJSON *root=cJSON_CreateObject(); 
	if(status)
	{
		cJSON_AddStringToObject(root,"cmdName","power_on"); 
	}
	else
	{
		cJSON_AddStringToObject(root,"cmdName","power_off"); 
	}
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_set_robot(unsigned char status)
{
	cJSON *root=cJSON_CreateObject(); 
	if(status)
	{
		cJSON_AddStringToObject(root,"cmdName","enable_robot"); 
	}
	else
	{
		cJSON_AddStringToObject(root,"cmdName","disable_robot"); 
	}
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_set_quit(void)
{
	cJSON *root=cJSON_CreateObject(); 
	cJSON_AddStringToObject(root,"cmdName","quit"); 
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;
}

int jaka_set_stop(void)
{
	cJSON *root=cJSON_CreateObject(); 
	cJSON_AddStringToObject(root,"cmdName","stop_program"); 
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_set_moveJ(double pose[6], double speed, double accel)
{
	cJSON * root, *list;
	root=cJSON_CreateObject(); 
	cJSON_AddStringToObject(root,"cmdName","joint_move"); 
	cJSON_AddItemToObject(root,"jointPosition", list = cJSON_CreateArray());
	for(int i=0;i<JOINT_MAX;i++)
	{
		cJSON_AddItemToArray(list, cJSON_CreateNumber(pose[i]));
	}
	if(speed==0)
	{
		speed = _default_speed;
	}
	cJSON_AddNumberToObject(root,"speed",speed);
	if(accel==0)
	{
		accel = _default_accel;
	}
	cJSON_AddNumberToObject(root,"accel",accel);
	cJSON_AddNumberToObject(root,"relFlag",0);
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;	
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_set_moveE(double pose[6], double speed, double accel)
{
	cJSON * root, *list;
	root=cJSON_CreateObject(); 
	cJSON_AddStringToObject(root,"cmdName","end_move"); 
	cJSON_AddItemToObject(root,"endPosition", list = cJSON_CreateArray());
	for(int i=0;i<JOINT_MAX;i++)
	{
		cJSON_AddItemToArray(list, cJSON_CreateNumber(pose[i]));
	}
	if(speed==0)
	{
		speed = _default_speed;
	}
	cJSON_AddNumberToObject(root,"speed",speed);
	if(accel==0)
	{
		accel = _default_accel;
	}
	cJSON_AddNumberToObject(root,"accel",accel);
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;	
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_set_moveL(double pose[6], double speed, double accel)
{
	cJSON * root, *list;
	root=cJSON_CreateObject(); 
	cJSON_AddStringToObject(root,"cmdName","moveL"); 
	cJSON_AddItemToObject(root,"cartPosition", list = cJSON_CreateArray());
	for(int i=0;i<JOINT_MAX;i++)
	{
		cJSON_AddItemToArray(list, cJSON_CreateNumber(pose[i]));
	}
	if(speed==0)
	{
		speed = _default_speed;
	}
	cJSON_AddNumberToObject(root,"speed",speed);
	if(accel==0)
	{
		accel = _default_accel;
	}
	cJSON_AddNumberToObject(root,"accel",accel);

	cJSON_AddNumberToObject(root,"relFlag",0);
	char *out = cJSON_Print(root);
	tcp_send(out,strlen(out));
	cJSON_Delete(root);  
    free(out); 
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_parse(char * msg)
{
	cJSON *json = cJSON_Parse(msg);
	cJSON *err = cJSON_GetObjectItem(json, "errorCode");
	cJSON *cmd = cJSON_GetObjectItem(json, "cmdName");
	DEBUG("cmd:%s err:%s\n",cmd->valuestring, err->valuestring);
	if(!strcmp("moveL", cmd->valuestring)&&!strcmp("0",err->valuestring))
	{
		jaka.resp.msgId = JAKA_MSG_MOVE_L;
	}
	else if(!strcmp("joint_move", cmd->valuestring)&&!strcmp("0",err->valuestring))
	{
		jaka.resp.msgId = JAKA_MSG_MOVE_J;
	}
	else if(!strcmp("end_move", cmd->valuestring)&&!strcmp("0",err->valuestring))
	{
		jaka.resp.msgId = JAKA_MSG_MOVE_E;
	}
	else if(!strcmp("power_on", cmd->valuestring)&&!strcmp("0",err->valuestring))
	{
		jaka.resp.msgId = JAKA_MSG_POWER_ON;
	}
	else if(!strcmp("enable_robot", cmd->valuestring)&&!strcmp("0",err->valuestring))
	{
		jaka.resp.msgId = JAKA_MSG_ROBOT_ENB;
	}
    else if(!strcmp("set_digital_output", cmd->valuestring)&&!strcmp("0",err->valuestring))
	{
		jaka.resp.msgId = JAKA_MSG_SET_DIO;
	}
	else
	{
		printf("failed.\r\n");
	}
	if(json)
	{
		cJSON_Delete(json);
	}
	
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_get_resp(void)
{
	int res = jaka.resp.msgId;
	jaka.resp.msgId = JAKA_MSG_NULL;
	return res;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int jaka_event(void)
{
	static int tim = 0;
	static int substate = 0;
	int ret = 0;
	switch(jaka.state)
	{
		case ROBOT_INIT:
			switch(substate)
			{
				case 0:
					jaka_set_power(JAKA_ON);
					substate++;
					printf("[jaka] start .");
					break;
				case 1:
					if (jaka_get_resp()==JAKA_MSG_POWER_ON)
					{
						substate++;
					}
					break;
				case 2:
					tim++;
					if(tim>=3)
					{
						tim = 0;
						substate++;
					}
					printf(".");
					break;
				case 3:
					jaka_set_robot(JAKA_ON);
					substate++;
					break;
				case 4:
					if (jaka_get_resp()==JAKA_MSG_ROBOT_ENB)
					{
						substate = 0;
						jaka.state++;
						printf(" ok\r\n");
                        //init_tio_do();
					}
					break;
				default:
					break;
			}
			break;
		case ROBOT_IDLE:
			break;
		case ROBOT_EXEC:
			break;
		case ROBOT_QUIT:
			break;
		default:
			break;
	}
	return ret;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
bool movel_callback(jaka_msgs::Move::Request &req, jaka_msgs::Move::Response &res)
{   
	if(ROBOT_IDLE==jaka.state)
	{
        jaka.state = ROBOT_EXEC;
		DEBUG("pose:[ ");
		for(int i=0; i<JOINT_MAX; i++)
		{
			jaka.end_pose[i] = (req.pose[i]);
			DEBUG("%f ", jaka.end_pose[i]);
		}
		DEBUG("]\r\n");

		double speed = (double)req.mvvelo;
		double accel = (double)req.mvacc;
		jaka_set_moveL(jaka.end_pose, speed, accel);

		res.ret = 1;
		res.message = "cmd has been executed!";
        jaka.state = ROBOT_IDLE;
		return true;
	}
	else
	{
		printf("jaka robot err.\r\n");
		return false;	
	}
}


bool movee_callback(jaka_msgs::Move::Request &req, jaka_msgs::Move::Response &res)
{   
	std::cout << "movee_callback....." << std::endl;
	if(ROBOT_IDLE==jaka.state)
	{
        jaka.state = ROBOT_EXEC;
		DEBUG("pose:[ ");
		for(int i=0; i<JOINT_MAX; i++)
		{
			jaka.end_pose[i] = (req.pose[i]);
			DEBUG("%f ", jaka.end_pose[i]);
		}
		DEBUG("]\r\n");

		double speed = (double)req.mvvelo;
		double accel = (double)req.mvacc;
		jaka_set_moveE(jaka.end_pose, speed, accel);

		res.ret = 1;
		res.message = "cmd has been executed!";
        jaka.state = ROBOT_IDLE;
		return true;
	}
	else
	{
		printf("jaka robot err.\r\n");
		return false;	
	}
}

/**
  * @brief 
  * @param 
  * @retval 
  */
bool movej_callback(jaka_msgs::Move::Request &req, jaka_msgs::Move::Response &res)
{   
	if(ROBOT_IDLE==jaka.state)
    {
        jaka.state = ROBOT_EXEC;
		DEBUG("joints:[ ");
		for(int i=0; i<JOINT_MAX; i++)
		{
			jaka.joints[i] = (req.pose[i]);
			DEBUG("%f ", jaka.joints[i]);
		}
		DEBUG("]\r\n");

		double speed = (double)req.mvvelo;
		double accel = (double)req.mvacc;
		jaka_set_moveJ(jaka.joints, speed, accel);

		res.ret = 1;
		res.message = "cmd has been executed!";
        jaka.state = ROBOT_IDLE;
		return true;
	}
	else
	{
		printf("jaka robot err.\r\n");
		return false;
	} 
}

/**
  * @brief 
  * @param 
  * @retval 
  */
bool stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	jaka_set_stop();
//	ROS_INFO("stop sucess 1");
//	while(1)
//	{
//		sleep(1);
//	}
//	ROS_INFO("stop sucess 2");
    return true;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
void gripper_Callback(std_msgs::UInt8 msg)
{
    if(ROBOT_IDLE==jaka.state)
	{
        jaka.state = ROBOT_EXEC;
	    if(msg.data)
        { 
            printf("[jaka] gripper close.\r\n");  
		    jaka_set_tio_do1(1);
	    }
        else
        {    
            printf("[jaka] gripper open.\r\n");
		    jaka_set_tio_do1(0);
		    jaka_set_tio_do2(1);
		    sleep(1);
		    jaka_set_tio_do2(0);
	    }
        jaka.state = ROBOT_IDLE;
    }
    else
    {
        printf("jaka robot err.\r\n");
    }
}

/**
  * @brief 
  * @param 
  * @retval 
  */
void* robot_respond(void* args){

    while (ros::ok())
    {
//		DEBUG("THREAD.\r\n"); 
        sleep(1);
    }
    
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int main(int argc, char**argv)
{
	ros::init(argc, argv, "jaka_controller");
  	ros::NodeHandle nh;
  	ros::Rate loop_rate(1);
	char rx_buf[RX_BUF_MAX];

	nh.getParam("/jaka_controller/default_speed", _default_speed);  
  	nh.getParam("/jaka_controller/default_accel", _default_accel);
	nh.getParam("/jaka_controller/ipv4", _ipv4);

	ros::ServiceServer service_movel = nh.advertiseService("/robot_driver/move_line", movel_callback);
	ros::ServiceServer service_movee = nh.advertiseService("/robot_driver/move_endpose", movee_callback);
	ros::ServiceServer service_movej = nh.advertiseService("/robot_driver/move_joint", movej_callback);
	ros::ServiceServer service_stop = nh.advertiseService("/robot_driver/stop_move",stop_callback);
	ros::Subscriber sub_gripper = nh.subscribe("/gripper1",1,gripper_Callback);

	pthread_t robot_state_thread;
    int ret = pthread_create(&robot_state_thread,NULL,robot_respond,NULL);
    if(ret!=0){
        ROS_ERROR("thread creat error!!!!!!!!!!");
    }

	if(tcp_init(_ipv4.c_str(), 10001))
	{
		exit(1);
	}

  	while (ros::ok())
  	{
		jaka_event();
		memset(rx_buf, 0, RX_BUF_MAX);
		if(tcp_recv(rx_buf, RX_BUF_MAX))
		{
			jaka_parse(rx_buf);
		}
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

	if(jaka.state)
	{
		jaka_set_quit();
		printf("quit\r\n");
	}

	tcp_deinit();
  	return 0;
}
