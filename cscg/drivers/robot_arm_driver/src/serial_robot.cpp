//
// Created by uav-robot on 19-1-8.
//


#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
#include <sstream>
#include "serial.h"

int fd;
void chatterCallback(const std_msgs::String msg)
{
    std::string middle_value;  //定义中间变量储存数据
    middle_value = msg.data;
    int  A_flag = middle_value.find('A');
    int  B_flag = middle_value.find('B');
    int  C_flag = middle_value.find('C');
    int  D_flag = middle_value.find('D');
    if(A_flag!=-1&&B_flag!=-1&&C_flag!=-1&&D_flag!=-1)
    {
        using Robot_arm_serial::Serial;
        Serial arm_Serial;
        tcflush(fd, TCIFLUSH);

        char tr_buffer1[middle_value.length()];
        std::strcpy(tr_buffer1,middle_value.c_str());

        unsigned char tr_buffer[middle_value.length()];

        std::memcpy(tr_buffer, tr_buffer1,  middle_value.length());
        arm_Serial.nwrite(fd, tr_buffer, middle_value.length());

    }
}

int main(int argc, char **argv)
{
    std::cout<<"hello arm!!"<<std::endl;
    int serial_port;

    // int RE_length, TR_length;
    ros::init(argc, argv, "robot_arm");
    ros::NodeHandle nh("~");
    if(!nh.getParam("serial_port",serial_port))
    {
        serial_port=0;
    }
    ros::Subscriber sub = nh.subscribe("/cscg/arm_control", 1000, chatterCallback); //订阅
    using Robot_arm_serial::Serial;
    Serial arm_Serial;

    fd = arm_Serial.open_port(serial_port);   //打开串口一
    if (fd == -1)
    {
        std::cout<<"open serail fail!"<<std::endl;
        return -1;
    }
    arm_Serial.set_opt(fd, 9600, 8, 'N', 1);   //设置串口参数
    ros::spin();	//程序进入循环，直到ros::ok()返回false，进程结束。
    arm_Serial.close_port(fd);
    return 0;
}