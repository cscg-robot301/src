//
// Created by weijin on 19-1-15.
//

#ifndef CSCG_HEX_POS_CONTROL_H
#define CSCG_HEX_POS_CONTROL_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/vehicle_position_setpoint.h>
#include <mavros_msgs/hex_constraints.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>

#include <Eigen/Core>

#include <iostream>
#include <cmath>

#include "hex_pos_control_parameters.h"

namespace cscg{
    namespace control {

        struct vehicleCurrentStates{

            geometry_msgs::Vector3 position;
            geometry_msgs::Vector3 velocity;
            geometry_msgs::Vector3 acceleration;
            float yaw;

        };
        class HexPositionControl{
        public:
            HexPositionControl();

            ~HexPositionControl() = default;

            void updateCurrentVehicleState(const vehicleCurrentStates& states);

            bool updateCurrentVehicleSetpoint(const mavros_msgs::vehicle_position_setpoint& setpoint);

            void updateConstraints(const mavros_msgs::hex_constraints &constraints);

        private:
            Eigen::Vector3f _pos{};      /// 无人机当前的位置
            Eigen::Vector3f _vel{};      /// 无人机当前的速度
            Eigen::Vector3f _velDot{};   /// 无人机当前速度的导数
            Eigen::Vector3f _acc{};      /// 无人机当前的加速度
            float _yaw{0.0f};            /// 无人机当前的偏航角


            Eigen::Vector3f _posSetPoint{};    /// 无人机位置设定点
            Eigen::Vector3f _velSetPoint{};    /// 无人机速度设定点
            Eigen::Vector3f _accSetPoint{};    /// 无人机加速度设定点
            Eigen::Vector3f _thrustSetPoint{}; /// 无人机推力设定点

            Eigen::Vector3f _thrustInit{};

            mavros_msgs::hex_constraints _constraints;

            HEXPosCtrl HEXC;
            float _yawSetPoint{};
            float _yawSpeedSetPoint{};


            bool _ctrlPosFlag[3] = {true, true, true};
            bool _ctrlVelFlag[3] = {true, true, true};




            void _positionController();
            void _velocityController(const float &dt);
            void _setCtrlFlagTrue();
            void _setCtrlFlagFalse();
            bool _interfaceMapping();
        };
    }
}

#endif //CSCG_HEX_POS_CONTROL_H
