#pragma once

#include <Arduino.h>

namespace MetaSense::CalibrationTask {

void begin();
void loop();

void tare();
void calibrate(float knownWeightKg);
void setSingle(const String& key, const String& value);
void resetAll();

} // namespace MetaSense::CalibrationTask