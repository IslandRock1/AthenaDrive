#include <stdio.h>
#include "PID.hpp"


PID_Reg::PID_Reg(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd){
}

void PID_Reg::setKp(float kp) {
    _kp = kp;
}

void PID_Reg::setKi(float ki) {
    _ki = ki;
}

void PID_Reg::setKd(float kd) {
    _kd = kd;
}

void PID_Reg::setSetpoint(float sp) {
    _setpoint = sp;
}

float PID_Reg::update(float currVal, float dt) {
    float error = _setpoint - currVal;

    _integral += error * dt;

    float derivative = 0.0;
    if (_firstRun) {
        _firstRun = false;
    } else if (dt != 0) {
        derivative = (error - _prevError) / dt;
    }

    _prevError = error;
    return _kp * error + _ki * _integral + _kd * derivative;
}