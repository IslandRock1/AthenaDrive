
#include "PI.hpp"

PI::PI(float kp, float ki)
    : _kp(kp), _ki(ki){
}

float PI::update(float error) {
    _integral += error;

    return _kp * error + _ki * _integral;
}