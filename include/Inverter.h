#pragma once

namespace MetaSense::Inverter {

float temperatureC();
void setTemperatureC(float tempC);

float torqueFeedForward();
void setTorqueFeedForward(float torque);

} // namespace MetaSense::Inverter
