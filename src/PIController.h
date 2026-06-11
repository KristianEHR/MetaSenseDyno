#pragma once

class PIController {
public:
    PIController();

    void setGains(float kp, float ki);
    void setOutputLimits(float outMin, float outMax);
    void reset();

    float compute(float setpoint, float measurement, float dtSec);

private:
    float _kp;
    float _ki;
    float _integral;
    float _outMin;
    float _outMax;
};
