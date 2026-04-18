#pragma once
// Using a separate PI controller will make the inner PI torque loop go faster.

class PI_Reg {
public:
    PI_Reg(float kp, float ki);

    void setKp(float kp);
    void setKi(float ki);

    float update(float error);

private:
    float _kp, _ki;
    float _integral = 0.0;
};