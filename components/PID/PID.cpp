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

    float derivative = 0.0f;
    if (!_firstRun && dt > 0.0f) {
        derivative = (error - _prevError) / dt;
    }
    _firstRun = false;

    float out = _kp * error + _ki * _integral + _kd * derivative;

    if (std::abs(out) > _maxOut) {
        _integral -= error * dt;    // anti-windup
        out = _maxOut * (out > 0 ? 1.0f : -1.0f);
    }

    _prevError = error;
    return out;
}