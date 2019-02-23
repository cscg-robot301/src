//
// Created by weijin on 19-1-18.
//
#include "ControlMath.h"

namespace cscg{
    namespace controlUtils{

        mavros_msgs::hex_attitude_setpoint thrustToAttitude(const Eigen::Vector3f &thrustSp, const float yawSp){

            mavros_msgs::hex_attitude_setpoint attSp = {};

            attSp.attitudeBody.z = yawSp;


            Eigen::Vector3f body_x, body_y, body_z;

            if(thrustSp.norm() > 0.00001f){

                body_z = thrustSp.normalized();

            } else{

                body_z.setZero();
                body_z(0) = 1.0f;

            }

            Eigen::Vector3f yaw_c(-sinf(attSp.attitudeBody.z), -cosf(attSp.attitudeBody.z), 0.0f);

            if(fabsf(body_z(2)) > 0.00001f){

                body_x = yaw_c.cross(body_z);

                if(body_z(2) < 0.0f){

                    body_x = - body_x;
                }

                body_x.normalize();

            } else{

                body_x.setZero();
                body_x(2) = 1.0f;
            }

            body_y = body_z.cross(body_x);

            Eigen::Matrix3d R_sp;

            for(int i = 0; i < 3; i++){

                R_sp(i, 0) = body_x(i);
                R_sp(i, 1) = body_y(i);
                R_sp(i, 2) = body_z(i);

            }

            Eigen::Quaterniond  qSP(R_sp);

            attSp.q.x = qSP.x();
            attSp.q.y = qSP.y();
            attSp.q.z = qSP.z();
            attSp.q.w = qSP.w();

            attSp.qValid = true;

            Eigen::Vector3d eulerAng = R_sp.eulerAngles(2, 1, 0);

            attSp.attitudeBody.x = eulerAng(0);
            attSp.attitudeBody.y = eulerAng(1);
            attSp.attitudeBody.z = eulerAng(2);

            attSp.thrustBody.z = -thrustSp.norm();

            return attSp;

        }

        Eigen::Vector2f constrainHori(const Eigen::Vector2f &var0, const Eigen::Vector2f &var1, const float &max){

            if((var0 + var1).norm() <= max){

                return var0 + var1;

            } else if(var0.norm() >= max){

                return var0.normalized() * max;

            } else if (fabsf((var1 - var0).norm()) < 0.001f){

                return var0.normalized() * max;

            } else if (fabsf(var0.norm()) < 0.001f){

                return var1.normalized() * max;
            } else{

                Eigen::Vector2f u1 = var1.normalized();
                float m = u1.dot(var0);
                float c = var0.dot(var0) - max * max;
                float s = -m + sqrtf(m * m - c);

                return var0 + u1 * s;

            }
        }

        bool crossSmoth(const Eigen::Vector3f &sphere_c, const float &sphere_r, const Eigen::Vector3f &line_a,
        const Eigen::Vector3f &line_b, Eigen::Vector3f &res)
        {
            Eigen::Vector3f ab_norm = line_a - line_b;
            if(ab_norm.norm() < 0.01f){

                return true;

            }

            ab_norm.normalize();
            Eigen::Vector3f d = line_a + ab_norm * ((sphere_c - line_a).dot(ab_norm));
            float cd_len = (sphere_c - d).norm();

            if(sphere_r > cd_len){

                float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);
                if((sphere_c - line_b).dot(ab_norm) > 0.0f){

                   res = line_b;


                }else{

                    res = d + ab_norm * dx_len;
                }

                return true;
            }else{

                res = d;

                if((sphere_c - line_a).dot(ab_norm) < 0.0f){

                    res = line_a;
                }

                if((sphere_c - line_b).dot(ab_norm) > 0.0f){

                    res = line_b;

                }

                return false;
            }
        }
    }
}