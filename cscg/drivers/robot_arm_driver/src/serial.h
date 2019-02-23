//
// Created by uav-robot on 19-1-4.
//

#ifndef PROJECT_SERAL_H
#define PROJECT_SERAL_H
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
namespace Robot_arm_serial{
    class Serial{
    public:
        Serial();
        ~Serial();
    public:
        static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
        static int open_port(int comport);
        static int open_port_laserdown();
        static int nwrite(int serialfd, unsigned char *data, int datalength);
        static int nread(int fd,  unsigned char *data, int datalength);
        static int close_port(int fd);
    };
}


#endif //PROJECT_SERAL_H
