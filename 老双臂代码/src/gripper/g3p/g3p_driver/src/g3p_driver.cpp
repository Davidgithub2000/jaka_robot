/**
  ******************************************************************************
  * File Name          : g3p_driver.cpp
  * Description       : g3p_driver program body
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
#include <g3p_driver/g3p_driver.h>

std::string _port;
std::string _sfile;
serial::Serial ser;
unsigned char last = 10;     //夹爪当前位置
unsigned char txd[TXD_MAX];  
//#if DRV8825
unsigned char gripper = 10;
//#endif                       

/* Private function prototypes -----------------------------------------------*/

  /**
  * @brief  夹爪位置读取
  * @param
  * @retval
  */
  int g3p_read(void)
  {
    int fd;
    unsigned char buf = 0; 
    ssize_t n;

    fd = open(_sfile.c_str(), O_RDONLY|O_CREAT,0777);
    if(fd == -1) 
    {
      printf("g3p_read error is %s\n", strerror(errno));
      return -1;
    }
    n = read(fd, &buf, sizeof(buf));
    close(fd);
    return buf;
  }

  /**
  * @brief  夹爪位置保存
  * @param
  * @retval
  */
  int g3p_write(unsigned char val)
  {
    int fd;
    unsigned char buf = val; 
    ssize_t n;

    fd = open(_sfile.c_str(), O_WRONLY);
    if(fd == -1) 
    {
      printf("g3p_write error is %s\n", strerror(errno));
      return -1;
    }
    n = write(fd, &buf, sizeof(buf));
    close(fd);
    return buf;
  }

#if DRV8825
#else
/**
  * @brief  通信数据校验和
  * @param
  * @retval
  */
  unsigned char checksum(unsigned char buf[TXD_MAX])
  {
    unsigned int cs=0;
    for(int i = 0; i<8; i++)
    {
      cs+=buf[3+i];
    }
    return (unsigned char)cs;
  }

/**
  * @brief  夹爪位置解析
  * @param
  * @retval
  */
void g3p_setPOS(unsigned char step)
{
  unsigned int tmp;

  if(step>STEP_MAX)
  {
    step = STEP_MAX;
  }

  txd[0] = 0xcc;        // 通信帧头
  txd[2] = 0x46;        // 运动速度

  if(step>last)
  {
    txd[1] = 0;
    tmp = (step-last)*MOTOR_1_8_STEP*DRIVER_SUBDIVIDE;
  }
  else if(step<last)
  {
    txd[1] = 1;
    tmp = (last-step)*MOTOR_1_8_STEP*DRIVER_SUBDIVIDE;
  }
  else
  {
    tmp = 0;
  }
  
  txd[3] = (unsigned char)tmp;
  txd[4] = (unsigned char)(tmp>>8);
  txd[5] = (unsigned char)(tmp>>16);
  txd[6] = (unsigned char)(tmp>>24);
  txd[7] = 0;
  txd[8] = 0;
  txd[9] = 0;
  txd[10] = 0;

  txd[11] = checksum(txd);

  last = step;
  if(g3p_write(last)>=0)
  {
    printf("txd:[");
    for(int i=0; i<TXD_MAX; i++)
    {
        printf(" %02x", txd[i]);
    }
    printf(" ]\r\n");
    printf("last = %d\r\n",last);
    ser.write(txd, TXD_MAX);
  }
}
#endif

/**
  * @brief  接收g3p_msgs/Gripper话题数据
  * @param  消息结构体指针
  * @retval 无
  */
void gripperCB(const g3p_msgs::Gripper::ConstPtr & msg)
{
#if DRV8825
    if((unsigned char)msg->state)
    {
      gripper = 1;
    }
    else
    {
      gripper = 0;
    }
#else
    gripper = (unsigned char)msg->state;
  //printf("gripperCB %d\r\n", gripper);
  if(gripper == last)
  {
    //夹爪状态相同，不用处理
  }
  else
  {
    g3p_setPOS(gripper);
  }
#endif
}

/**
  * @brief  柔性三指驱动主函数
  * @param
  * @retval
  */
int main(int argc, char**argv)
{
  ros::init(argc, argv, "g3p_driver");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  nh.getParam("/g3p_driver/port", _port);
  nh.getParam("/g3p_driver/sfile", _sfile);

  //订阅夹爪话题
  ros::Subscriber sub_gripper = nh.subscribe("g3p_msgs/gripper", 100, gripperCB);

  try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort(_port.c_str()); 
#if DRV8825
        ser.setBaudrate(9600); 
#else
        ser.setBaudrate(115200); 
#endif
        serial::Timeout to = serial::Timeout::simpleTimeout(5); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    last = g3p_read();
    gripper = last;
    printf("gripper:%d\r\n",last);

  while (ros::ok())
  {
#if DRV8825
    if(gripper!=last)
    {
      printf("gripper:%d\r\n",gripper);
      txd[0] = 0x01;
      txd[1] = 0x06;
      txd[2] = 0x00;
      txd[3] = 0x0b;
      txd[4] = 0x00;
      // 方向判断
      if(gripper)
      { //闭合  01 06 00 0b 00 00 F8 08
        txd[5] = 0x00;
        txd[6] = 0xf8;
        txd[7] = 0x08;
      }
      else
      { //张开  01 06 00 0b 00 01 39 C8
        txd[5] = 0x01;
        txd[6] = 0x39;
        txd[7] = 0xc8;
      }
      ser.write(txd, TXD_MAX);
      usleep(50000);
      // 单次运行
      txd[0] = 0x01;
      txd[1] = 0x05;
      txd[2] = 0x00;
      txd[3] = 0x07;
      txd[4] = 0xff;
      txd[5] = 0x00;
      txd[6] = 0x3d;
      txd[7] = 0xfb;
      last = gripper;
      ser.write(txd, TXD_MAX);
      g3p_write(last);
    }
#endif
    ros::spinOnce();
    loop_rate.sleep();
  }

  if(ser.isOpen())
  {
    ser.close();
  }
  
  return 0;
}
