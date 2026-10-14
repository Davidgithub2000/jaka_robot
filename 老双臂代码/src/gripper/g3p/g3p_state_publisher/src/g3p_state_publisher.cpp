/**
  ******************************************************************************
  * File Name          : g3p_state_publisher.cpp
  * Description       : g3p_state_publisher program body
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
#include <g3p_state_publisher/g3p_state_publisher.h>

#define theta_0   1.2655 
#define theta_1   1.5708
#define theta_2   1.5907
#define theta_3   1.8561

double last = 0;
double theta[4] ;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  接收g3p_msgs/Gripper话题数据
  * @param  消息结构体指针
  * @retval 无
  */
void gripperCB(const g3p_msgs::Gripper::ConstPtr & msg)
{
  double gripper = (unsigned char)msg->state;
#if DRV8825
  if(gripper>0)
  {
    gripper = 11;
  }
#endif
  //printf("gripperCB %d\r\n", gripper);
  if(gripper == last)
  {
    //夹爪状态相同，不用处理
  }
  else
  {
    gripper3d(gripper, theta);
    last = gripper;
    printf(" t1:%0.4f,  t2:%0.4f, t3:%0.4f t4:%0.4f\r\n",theta[0], theta[1], theta[2], theta[3]);
    theta[0] = theta[0]-theta_0;
    theta[1] = theta[1]-theta_1;
    theta[2] = theta[2]-theta_2;
    theta[3] = theta[3]-theta_3;
  }
}

/**
  * @brief  柔性三指夹爪状态主函数
  * @param
  * @retval
  */
int main(int argc, char**argv)
{
  ros::init(argc, argv, "g3p_state_publisher");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  sensor_msgs::JointState joint_state;
  nh.getParam("/g3p_state_publisher/joint_names", joint_state.name);
  nh.getParam("/g3p_state_publisher/joint_position", joint_state.position);
  ros::Publisher pub_states = nh.advertise<sensor_msgs::JointState>("joint_states", 100);

  //订阅夹爪话题
  ros::Subscriber sub_gripper = nh.subscribe("g3p_msgs/gripper", 100, gripperCB);

  while (ros::ok())
  {
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = last/1000;    //单位转换cm->m

    joint_state.position[1] = theta[2];
    joint_state.position[2] = -1*theta[3];

    joint_state.position[3] = theta[2];
    joint_state.position[4] = -1*theta[3];

    joint_state.position[5] = theta[2];
    joint_state.position[6] = -1*theta[3];
 
    pub_states.publish(joint_state);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
