#pragma once

class PID_Reg {
public:
    PID_Reg(float kp, float ki, float kd);

    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    
    void setSetpoint(float sp);
    void setMaxOut(float maxOut) { _maxOut = std::abs(maxOut); }
    float update(float currVal, float dt);

private:
    float _kp, _ki, _kd;
    float _setpoint = 0.0;
    float _integral = 0.0;
    float _prevError = 0.0;
    float _maxOut = 100.0;
    bool _firstRun = true;
};