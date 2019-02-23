//// Created by weijin on 19-1-14.//#include "hex_pos_control.h"#include "ControlMath.h"#include "Limits.h"namespace cscg{    namespace control{        HexPositionControl::HexPositionControl() {        }        void HexPositionControl::updateCurrentVehicleState(const vehicleCurrentStates &states) {            _pos    << states.position.x, states.position.y, states.position.z;            _vel    << states.velocity.x, states.velocity.y, states.velocity.z;            _velDot << states.acceleration.x, states.acceleration.y, states.acceleration.z;            _yaw    = states.yaw;        }        bool HexPositionControl::updateCurrentVehicleSetpoint(const mavros_msgs::vehicle_position_setpoint &setpoint) {            _setCtrlFlagTrue();            _posSetPoint        << setpoint.position.x, setpoint.position.y, setpoint.position.z;            _velSetPoint        << setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z;            _accSetPoint        << setpoint.acceleration.x, setpoint.acceleration.y, setpoint.acceleration.z;            _thrustSetPoint     << setpoint.thrust.x, setpoint.thrust.y, setpoint.thrust.z;            _yawSetPoint        = setpoint.yaw;            _yawSpeedSetPoint   = setpoint.yawSpeed;        }        void HexPositionControl::updateConstraints(const mavros_msgs::hex_constraints &constraints) {            _constraints = constraints;        }        void HexPositionControl::_setCtrlFlagTrue() {            for(int i = 0; i < 3; i++){                _ctrlPosFlag[i] = true;                _ctrlVelFlag[i] = true;            }        }        void HexPositionControl::_setCtrlFlagFalse() {            for(int i = 0; i < 3; i++){                _ctrlPosFlag[i] = false;                _ctrlVelFlag[i] = false;            }        }        bool HexPositionControl::_interfaceMapping() {            bool failsafe = false;            for(int i = 0; i < 3; i++){                if(std::isfinite(_posSetPoint(i))){                    if(!std::isfinite(_velSetPoint(i))){                        _velSetPoint(i) = 0.0f;                    }                    /// 在位置控制模式下，推力设定没有                    _thrustSetPoint(i) = NAN;                    /// 位置控制要求有效的位置和速度                    if(!std::isfinite(_pos(i)) || !std::isfinite(_vel(i))){                        failsafe = true;                    }                } else if (std::isfinite(_velSetPoint(i))){                    /// 如果位置无效，且速度有效，因此速度控制器有效                    _posSetPoint(i) = _pos(i) = 0.0f;                    /// 屏蔽位置控制器                    _ctrlPosFlag[i] = false;                    if(!std::isfinite(_vel(i))){                        failsafe = false;                    }                } else if (std::isfinite(_thrustSetPoint(i))){                    _posSetPoint(i) = _pos(i) = 0.0f;                    _velSetPoint(i) = _vel(i) = 0.0f;                    _ctrlPosFlag[i] = _ctrlVelFlag[i] = false;                    _thrustInit(i) = 0.0f;                    _velDot(i) = 0.0f;                } else{                    failsafe = true;                }            }            if(failsafe){                _thrustSetPoint(0) = _thrustSetPoint(1) = 0.0f;                _thrustSetPoint(2) = -(HEXC.HEX_THR_MIN + (HEXC.HEX_THR_HOVER - HEXC.HEX_THR_MIN) * 0.7);                _setCtrlFlagFalse();            }            return (!failsafe);        }        void HexPositionControl::_positionController() {            const Eigen::Vector3f velSP = (_posSetPoint - _pos) * HEXC.HEX_HRI_P;            _velSetPoint = velSP + _velSetPoint;            Eigen::Vector2f  velSPTemp(velSP(0), velSP(1));            Eigen::Vector2f  velSPTemp1((_velSetPoint - velSP)(0), (_velSetPoint - velSP)(1));           const Eigen::Vector2f velSpHori = controlUtils::constrainHori(velSPTemp, velSPTemp1,  _constraints.speedHori);           _velSetPoint(0) = velSpHori(0);           _velSetPoint(1) = velSpHori(1);           _velSetPoint(2) = math::constrain(_velSetPoint(2), -_constraints.speedUp, _constraints.speedDown);        }       void HexPositionControl::_velocityController(const float &dt){            const Eigen::Vector3f velError = _velSetPoint - _vel;            float thrustDD = HEXC.HEX_Z_VEL_P * velError.z() + HEXC.HEX_Z_VEL_D * _velDot(2) + _thrustInit(2)                    + HEXC.HEX_THR_HOVER;            float uMax = -HEXC.HEX_THR_MIN;            float uMin = -HEXC.HEX_THR_MAX;            bool stopIntegralD = ((thrustDD >= uMax && velError(2) >= 0.0f) ||                    (thrustDD <= uMin && velError(2) <= 0.0f));            if(!stopIntegralD){                _thrustInit(2) += velError(2) + HEXC.HEX_Z_VEL_I * dt;                _thrustInit(2) = math::min(_thrustInit(2), HEXC.HEX_THR_MAX * std::sinh(_thrustInit(2)));            }            _thrustSetPoint(2) = math::constrain(thrustDD, uMin, uMax);            if(std::isfinite(_thrustSetPoint(0)) && std::isfinite(_thrustSetPoint(1))){               float thrustMaxHori = std::fabs(_thrustSetPoint(2)) * std::tan(_constraints.tilt);               _thrustSetPoint(0) *= thrustMaxHori;               _thrustSetPoint(1) *= thrustMaxHori;            } else{                Eigen::Vector2f thrustDNE;                thrustDNE(0) = HEXC.HEX_XY_VEL_P * velError(0) + HEXC.HEX_XY_VEL_D * _velDot(0) + _thrustInit(0);                thrustDNE(1) = HEXC.HEX_XY_VEL_P * velError(1) + HEXC.HEX_XY_VEL_D * _velDot(1) + _thrustInit(1);                float thrustMaxNETilt = fabs(_thrustSetPoint(2)) * std::tan(_constraints.tilt);                float thrustMaxNE = sqrtf(HEXC.HEX_THR_MAX * HEXC.HEX_THR_MAX - _thrustSetPoint(2) * _thrustSetPoint(2));                thrustMaxNE = math::min(thrustMaxNETilt, thrustMaxNE);                _thrustSetPoint(0) = thrustDNE(0);                _thrustSetPoint(1) = thrustDNE(1);                if(thrustDNE.dot(thrustDNE) > thrustMaxNE * thrustMaxNE){                    float mag = thrustDNE.norm();                    _thrustSetPoint(0) = thrustDNE(0) / mag * thrustMaxNE;                    _thrustSetPoint(1) = thrustDNE(1) / mag * thrustMaxNE;                }                float arwGain = 2.f / HEXC.HEX_XY_VEL_P;                Eigen::Vector2f velErrorLim;                velErrorLim(0) = velError(0) - (thrustDNE(0) - _thrustSetPoint(0) * arwGain);                velErrorLim(1) = velError(1) - (thrustDNE(1) - _thrustSetPoint(1) * arwGain);                _thrustInit(0) += HEXC.HEX_XY_VEL_I * velErrorLim(0) * dt;                _thrustInit(1) += HEXC.HEX_XY_VEL_I * velErrorLim(1) * dt;            }        }    }}