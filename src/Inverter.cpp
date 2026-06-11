#include "Inverter.h"

namespace {

float inverterTempC = 0.0f;
float torqueFeedForwardNm = 0.0f;

} // anonymous namespace


namespace MetaSense::Inverter {

float temperatureC()
{
    return inverterTempC;
}

void setTemperatureC(float tempC)
{
    inverterTempC = tempC;
}

float torqueFeedForward()
{
    return torqueFeedForwardNm;
}

void setTorqueFeedForward(float torque)
{
    torqueFeedForwardNm = torque;
}

} // namespace MetaSense::Inverter
