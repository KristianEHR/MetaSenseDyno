#pragma once

namespace MetaSense::ControlTask {

void begin();
void loop();

void configurePI(float kp, float ki, float outMin, float outMax);
void resetPI();
float computeTorqueCommand(float rpmTarget, float rpmActual, float dtSec);

} // namespace MetaSense::ControlTask
