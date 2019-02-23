//
// Created by weijin on 19-1-8.
//
#include "yesense.h"

namespace drivers{
    namespace yesense{

        Yesense::Yesense(const std::string port, u_int32_t baubrate, std::string FrameId) {

            imu.header.frame_id = FrameId;
            yesense_serial.setPort(port);
            yesense_serial.setBaudrate(baubrate);
            //time_t tt = time(NULL);
            //tm* t = localtime(&tt);
            //sprintf(data_path_, "/home/weijin/CSCG_UAV/imu.txt");


        }

        Yesense::~Yesense() {

            yesense_serial.close();
        }

        bool Yesense::checkSerialOpen() {
            if(yesense_serial.isOpen()){

                ROS_ERROR("### Another application is using the serial port, please close it!!!");
                yesense_serial.close();
            }
            yesense_serial.open();
            return true;
        }

        void Yesense::processDataToImu() {

            unsigned char headAndTail[7];
            unsigned char data[6][18];

            yesense_serial.read(headAndTail, 2);
            //yesense_serial.read(headAndTail, 3);
            if(headAndTail[0] == 0x59 && headAndTail[1] == 0x53)
            {
                readData(3, &headAndTail[2]);
                //std::cout << headAndTail[2] << std::endl;
                mutex = true;
               for(int i = 0; i < 5; i++){

                    readData(14, data[i]);
               }
               readData(18, data[5]);
               readData(2, &headAndTail[5]);
               for(int j = 0; j < 6; j++){

                   DecodeImuData(data[j]);
                }

            }
        }

        long Yesense::get_signed_int(unsigned char *data) {

            int temp = 0;
            temp = long((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
            return temp;

        }

        void Yesense::DecodeImuData(unsigned char *reTemp) {

            switch (reTemp[0])
            {
                case ACCEL_ID:
                    acc[0] = get_signed_int(reTemp) * NOT_MAG_DATA_FACTOR;
                    acc[1] = get_signed_int(reTemp + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                    acc[2] = get_signed_int(reTemp + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
                    break;
                case ANGLE_ID:
                    gyro[0] = get_signed_int(reTemp) * NOT_MAG_DATA_FACTOR;
                    gyro[1] = get_signed_int(reTemp + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                    gyro[2] = get_signed_int(reTemp + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
                    break;
                case MAGNETIC_ID:
                    mag[0] = get_signed_int(reTemp) * NOT_MAG_DATA_FACTOR;
                    mag[1] = get_signed_int(reTemp + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                    mag[2] = get_signed_int(reTemp + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
                    break;
                case RAW_MAGNETIC_ID:
                    raw_mag[0] = get_signed_int(reTemp) * MAG_RAW_DATA_FACTOR;
                    raw_mag[1] = get_signed_int(reTemp + SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
                    raw_mag[2] = get_signed_int(reTemp + SINGLE_DATA_BYTES * 2) * MAG_RAW_DATA_FACTOR;
                    break;
                case EULER_ID:
                    euler[0] = get_signed_int(reTemp) * NOT_MAG_DATA_FACTOR;
                    euler[1] = get_signed_int(reTemp + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                    euler[2] = get_signed_int(reTemp + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
                    break;
                case QUATERNION_ID:
                    quaternion[0] = get_signed_int(reTemp) * NOT_MAG_DATA_FACTOR;
                    quaternion[1] = get_signed_int(reTemp + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                    quaternion[2] = get_signed_int(reTemp + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
                    quaternion[3] = get_signed_int(reTemp + SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
                    break;
                default:
                    break;

            }
        }

        void Yesense::readData(int size, unsigned char *data) {
            int head = 0;
            int end = 0;
            int size_temp = size;
            while (head < size){
                end = yesense_serial.read(&data[head], size_temp);
                if(end == size_temp)
                    head += size_temp;
                else if(end != EOF){
                    int mid = head;
                    head += end;
                    if((head + size_temp) >= size)
                        size_temp = size-head;
                }
            }
        }

        void Yesense::printData() {

            if(mutex){
                std::cout << " *************************imu data**********************" << std::endl;
                std::cout << "acc: " << acc[0] << ", "<<acc[1] << ", " << acc[2] << std::endl;
                std::cout << "gyro: " << gyro[0] << ", "<< gyro[1] << ", " << gyro[2] << std::endl;
                std::cout << "mag: " << mag[0] << ", "<< mag[1] << ", " << mag[2] << std::endl;
                std::cout << "raw_mag: " << raw_mag[0] << ", "<< raw_mag[1] << ", " << raw_mag[2] << std::endl;
                std::cout << "euler: " << euler[0] << ", "<< euler[1] << ", " << euler[2] << std::endl;
                std::cout << "quaternion: " << quaternion[0] << ", "<< quaternion[1] << ", " << quaternion[2] << ", " << quaternion[3] << std::endl;

                /// *************************** 存储数据 ****************************************///

            }


        }

        void Yesense::getImuMessage(sensor_msgs::Imu &raw_imu) {

            raw_imu.header.frame_id = "cscg_imu";
            raw_imu.header.stamp = ros::Time::now();
            raw_imu.linear_acceleration.x = acc[0];
            raw_imu.linear_acceleration.y = acc[1];
            raw_imu.linear_acceleration.z = acc[2];
            raw_imu.linear_acceleration_covariance[0] = acc[0];
            raw_imu.linear_acceleration_covariance[4] = acc[1];
            raw_imu.linear_acceleration_covariance[8] = acc[2];

            raw_imu.angular_velocity.x = gyro[0];
            raw_imu.angular_velocity.y = gyro[1];
            raw_imu.angular_velocity.z = gyro[2];
            raw_imu.angular_velocity_covariance[0] = gyro[0];
            raw_imu.angular_velocity_covariance[4] = gyro[1];
            raw_imu.angular_velocity_covariance[8] = gyro[2];

            raw_imu.orientation.w = quaternion[0];
            raw_imu.orientation.x = quaternion[1];
            raw_imu.orientation.y = quaternion[2];
            raw_imu.orientation.z = quaternion[3];


        }




    }
}