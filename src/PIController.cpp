#include "PIController.h"

namespace {

float clamp(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

} // anonymous namespace


PIController::PIController()
    : _kp(0.0f),
      _ki(0.0f),
      _integral(0.0f),
      _outMin(-1.0f),
      _outMax(1.0f)
{
}

void PIController::setGains(float kp, float ki)
{
    _kp = kp;
    _ki = ki;
}

void PIController::setOutputLimits(float outMin, float outMax)
{
    _outMin = outMin;
    _outMax = outMax;
    _integral = clamp(_integral, _outMin, _outMax);
}

void PIController::reset()
{
    _integral = 0.0f;
}

float PIController::compute(float setpoint, float measurement, float dtSec)
{
    const float error = setpoint - measurement;

    _integral += error * _ki * dtSec;
    _integral = clamp(_integral, _outMin, _outMax);

    const float output = _kp * error + _integral;
    return clamp(output, _outMin, _outMax);
}
