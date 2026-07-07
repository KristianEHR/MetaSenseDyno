#pragma once

#include "Telemetry.h"
#include <Arduino.h>

namespace MetaSense::RunStorage {

// Live telemetry snapshot
void save(const MetaSense::Telemetry& telemetry);
MetaSense::Telemetry latest();
uint32_t version();
void setPublishTaskHandle(TaskHandle_t handle);
void flush();
void saveCalibration();
bool loadCalibration(float& zeroOffset, float& calibrationFactor);

// Raw capture for quality analysis
bool startRawCapture(uint32_t durationMs, float baselineKg, float testLoadKg, String& outFilePath);
void appendRawCaptureSample(uint64_t timestampUs, float rawValue, float filteredValue, const char* sourceLabel);
void tickRawCapture(uint64_t nowUs);
bool rawCaptureActive();
String rawCaptureStateJson();
String listRawCaptures();
String loadRawCaptureReport(const String& filename);
String verifyRawCaptureCsv(const String& filename);

// Minimal live data FS probe (store/read validation with runtime samples)
bool startFsLiveProbe(uint32_t durationMs, String& outFilePath);
void appendFsLiveProbeSample(uint64_t timestampUs, float value);
void tickFsLiveProbe(uint64_t nowUs);
String fsLiveProbeStateJson();
String fsLiveProbeReadText();
String fsLiveProbeVerifyJson();

// Persistent run history (LittleFS)
bool saveRun(const String& payload);           // payload = JSON after "SAVE_RUN_DATA:"
String listRuns();                             // returns JSON array of run metadata objects
String loadRunPoints(const String& filename);  // returns JSON points array for one run
bool deleteRunByIndex(int index);              // delete the nth run in sorted order

} // namespace MetaSense::RunStorage