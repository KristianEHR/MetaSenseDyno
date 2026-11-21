#include "CalibrationTask.h"

#include <Arduino.h>

#include "Notify.h"

namespace {

int32_t tareRaw = 0;
float scaleFactor = 1.0f;

} // anonymous namespace


namespace MetaSense::CalibrationTask {

void begin()
{
    // could load from NVS if needed
}

void loop()
{
    // optional drift compensation or periodic checks
}

void tare()
{
    int32_t raw = (int32_t)analogRead(32);
    tareRaw = raw;
    MetaSense::Notify::sendInfo("Load cell tare completed");
}

void calibrate(float refKg)
{
    int32_t raw = (int32_t)analogRead(32);
    int32_t net = raw - tareRaw;

    if (net == 0) {
        MetaSense::Notify::sendError("Calibration failed (no load change)");
        return;
    }

    scaleFactor = refKg / (float)net;
    MetaSense::Notify::sendStatus("Load cell calibration OK");
}

void setSingle(const String& key, const String& value)
{
    (void)key;
    (void)value;
}

void resetAll()
{
    tareRaw = 0;
    scaleFactor = 1.0f;
    MetaSense::Notify::sendStatus("Calibration reset");
}

} // namespace MetaSense::CalibrationTask