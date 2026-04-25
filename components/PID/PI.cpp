
#include "PI.hpp"

PI_Reg::PI_Reg(float kp, float ki)
    : _kp(kp), _ki(ki){
}

float PI_Reg::update(float error) {
    _integral += error;

    return _kp * error + _ki * _integral;
}

void PI_Reg::setKp(float kp) {
    _kp = kp;
}

void PI_Reg::setKi(float ki) {
    _ki = ki;
}