#ifndef __JAKA_TCP_H__
#define __JAKA_TCP_H__

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>

int tcp_recv(char *buffer, unsigned int size);
int tcp_send(const char *buffer, unsigned int size);
int tcp_init(const char * ip_addr, unsigned int ip_port);
void tcp_deinit(void);

#endif //__JAKA_TCP_H__
