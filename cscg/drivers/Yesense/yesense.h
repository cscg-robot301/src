//
// Created by weijin on 19-1-8.
//

#ifndef CSCG_YESENSE_H
#define CSCG_YESENSE_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

namespace drivers{
    namespace yesense{

#define ACCEL_ID				(unsigned char)0x10
#define ANGLE_ID				(unsigned char)0x20
#define MAGNETIC_ID				(unsigned char)0x30     /*归一化值*/
#define RAW_MAGNETIC_ID			(unsigned char)0x31     /*原始值*/
#define EULER_ID				(unsigned char)0x40
#define QUATERNION_ID			(unsigned char)0x41
#define UTC_ID					(unsigned char)0x50
#define LOCATION_ID				(unsigned char)0x60
#define SPEED_ID				(unsigned char)0x70

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR			0.000001f
#define MAG_RAW_DATA_FACTOR			0.001f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR		0.0000001
#define ALT_DATA_FACTOR				0.001f
#define SPEED_DATA_FACTOR			0.001f

#define SINGLE_DATA_BYTES				4

        class Yesense{
        public:
            Yesense(const std::string port, u_int32_t baubrate, std::string FrameId);
            ~Yesense();

            bool checkSerialOpen();
            void processDataToImu();
            long get_signed_int(unsigned char *data);
            void DecodeImuData(unsigned char *reTemp);
            void readData(int size, unsigned char *data);
            void printData();
            void getImuMessage(sensor_msgs::Imu &raw_imu);

        private:

            sensor_msgs::Imu imu;
            serial::Serial yesense_serial;
            float acc[3];
            float gyro[3];
            float mag[3];
            float raw_mag[3];
            float euler[3];
            float quaternion[4];
        public:
            bool mutex = false;
            //std::ofstream  data("/home/weijin/CSCG_UAV/imu.txt");


        };
    }
}
#endif //CSCG_YESENSE_H
