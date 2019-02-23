#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
#include <sstream>
#include "serial.h"

#include <geometry_msgs/Vector3.h>

/* 接受到期望位置和姿态 已经当前位置和姿态 进行调整*/

using namespace std;

void chatterCallback_first(const geometry_msgs::Vector3 msg)
{
   // ROS_INFO("node_b is receiving [%s]", msg->data.c_str());
    cout<<"Hello World [%s]"<< endl;
}

void chatterCallback_second(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("node_b is receiving [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{
    std::cout<<"hello arm!!"<<std::endl;

    ros::init(argc, argv, "robot_decision"); //初始化一个ROS节点命名为  robot_desicion

    ros::NodeHandle n;	                     //实例化节点, 节点进程句柄
    ros::Publisher pub = n.advertise<std_msgs::String>("/cscg/arm_control", 1000);
    //告诉系统要发布话题了，话题名为“str_message”，类型为std_msgs::String，缓冲队列为1000

    ros::Subscriber get_desire_pos_sub = n.subscribe("desire_message", 1000, chatterCallback_first);
    //向话题“str_message”订阅，一旦发布节点（node_a）在该话题上发布消息，本节点就会调用chatterCallbck函数。

    ros::Subscriber get_current_pos_sub = n.subscribe("current_message", 1000, chatterCallback_second);
    //向话题“str_message”订阅，一旦发布节点（node_a）在该话题上发布消息，本节点就会调用chatterCallbck函数。



    ros::Rate loop_rate(1);	//设置发送数据的频率为10Hz

    while(ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss<<"A60B60C60D60";

        msg.data = ss.str();

        ROS_INFO("node_a is publishing %s", msg.data.c_str());
        pub.publish(msg);	//向话题“str_message”发布消息
        loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起

        std::stringstream dd;
        dd<<"A90B90C90D90";

        msg.data = dd.str();

        ROS_INFO("node_a is publishing %s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();	//不是必须，若程序中订阅话题则必须，否则回掉函数不起作用。
        loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
    }

    return 0;

}