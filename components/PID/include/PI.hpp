#pragma once
// Using a separate PI controller will make the inner PI torque loop go faster.

class PI {
public:
    PI(float kp, float ki);
    float update(float error);

private:
    float _kp, _ki;
    float _integral = 0.0;
};