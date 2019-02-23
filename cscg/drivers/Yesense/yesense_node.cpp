//
// Created by weijin on 19-1-8.
//

#include "yesense.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char * argv[]){

    ros::init(argc, argv, "yesense_node");
    ros::NodeHandle nh("~");
    std::string port;
    std::string frame_id;
    int baubrate;
    sensor_msgs::Imu raw_imu;
    if(!nh.getParam("serial_port", port)){
        port = "/dev/ttyUSB0";
    }

    if(!nh.getParam("baubrate", baubrate)){
        baubrate = 460800;
    }

    if(!nh.getParam("frame_id", frame_id)){
        frame_id = "yesense_imu";
    }

    ros::Publisher rawImuPub = nh.advertise<sensor_msgs::Imu>("/CSCG/rawImu", 10);

    drivers::yesense::Yesense sense(port, baubrate, frame_id);
    sense.checkSerialOpen();
    ros::Rate loopRate(50);
    while (ros::ok()){
        sense.processDataToImu();
        sense.getImuMessage(raw_imu);
        if(sense.mutex){
            rawImuPub.publish(raw_imu);
            sense.printData();
            sense.mutex = false;
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}