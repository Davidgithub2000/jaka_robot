/**
  ******************************************************************************
  * File Name          : jaka_tcp.cpp
  * Description       : jaka_tcp program body
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
#include <jaka_feedback/jaka_tcp.h>

using namespace std;

static int server_sockfd;
struct sockaddr_in server_addr;

#ifdef TEST
int sysUsecTime() 
{ 
    struct timeval    	tv; 
    struct timezone 	tz;  
    struct tm         	*p; 
       
    gettimeofday(&tv, &tz); 
    //printf("tv_sec:%ld\n",tv.tv_sec); 
    //printf("tv_usec:%ld\n",tv.tv_usec); 
    //printf("tz_minuteswest:%d\n",tz.tz_minuteswest); 
    //printf("tz_dsttime:%d\n",tz.tz_dsttime); 
       
    p = localtime(&tv.tv_sec); 
    printf("%d.%d.%d %d:%d:%d:[%ld]\n",1900+p->tm_year,1+p->tm_mon,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec, tv.tv_usec);
    return tv.tv_usec; 
} 
#endif

int tcp_recv(char *buffer, unsigned int size)
{
	int fs_sel;
	int n;
    fd_set  fs_read;
    struct timeval tv_timeout;
	unsigned int server_addr_len;
	
	FD_ZERO(&fs_read);
	FD_SET(server_sockfd, &fs_read);
	tv_timeout.tv_sec = 2;
	tv_timeout.tv_usec = 0;
	fs_sel = select(server_sockfd+1, &fs_read, NULL, NULL, &tv_timeout);
	if(fs_sel)
	{
		n = recv(server_sockfd, buffer, size, 0);
		if(n>0)
		{
#ifdef TEST
			sysUsecTime();
#endif
			return 1;
		}
		else 
		{
			if(n==0)
			{
				printf("[ROBOT]：client close\r\n");
			}
			else
			{
				printf("[ROBOT]：client error\r\n");
			}
	
			if(server_sockfd)
			{
				printf("[ROBOT]：server IP[%s]\r\n",inet_ntoa(server_addr.sin_addr));
				close(server_sockfd);
			}
		}
	}
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int tcp_send(const char *buffer, unsigned int size)
{
	return send(server_sockfd, buffer, size, 0);
}

/**
  * @brief 
  * @param 
  * @retval 
  */
int tcp_init(const char * ip_addr, unsigned int ip_port)
{

	server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
	int sendbytes;
	server_addr.sin_port = htons(ip_port); 
	server_addr.sin_addr.s_addr  = inet_addr(ip_addr); 
	server_addr.sin_family = AF_INET;
	int len = sizeof(sockaddr_in);
	if(connect(server_sockfd, (sockaddr*)&server_addr, len) == -1)
	{
		cout << "connect error:"  << endl;
		return 1;
	}
    printf("connect port:[%d] -> ok! \n", ip_port);
	return 0;
}

/**
  * @brief 
  * @param 
  * @retval 
  */
void tcp_deinit(void)
{
	if(server_sockfd)
	{	
		close(server_sockfd);
	}
}

