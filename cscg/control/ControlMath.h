//
// Created by weijin on 19-1-18.
//

#ifndef CSCG_CONTROLMATH_H
#define CSCG_CONTROLMATH_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <mavros_msgs/hex_attitude_setpoint.h>

namespace cscg{
    namespace controlUtils{

        mavros_msgs::hex_attitude_setpoint thrustToAttitude(const Eigen::Vector3f &thrustSp, const float yawSp);

        Eigen::Vector2f constrainHori(const Eigen::Vector2f &var0, const Eigen::Vector2f &var1, const float &max);

        bool crossSmooth(const Eigen::Vector3f &sphere_c, const float &sphere_r, const Eigen::Vector3f &line_a,
                const Eigen::Vector3f &line_b, Eigen::Vector3f &res);


    }
}
#endif //CSCG_CONTROLMATH_H
