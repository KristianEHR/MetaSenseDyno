#pragma once

#include "Telemetry.h"
#include <Arduino.h>

namespace MetaSense::RunStorage {

// Live telemetry snapshot
void save(const MetaSense::Telemetry& telemetry);
const MetaSense::Telemetry& latest();
void flush();
void saveCalibration();

// Persistent run history (LittleFS)
bool saveRun(const String& payload);           // payload = JSON after "SAVE_RUN_DATA:"
String listRuns();                             // returns JSON array of run metadata objects
String loadRunPoints(const String& filename);  // returns JSON points array for one run
bool deleteRunByIndex(int index);              // delete the nth run in sorted order

} // namespace MetaSense::RunStorage