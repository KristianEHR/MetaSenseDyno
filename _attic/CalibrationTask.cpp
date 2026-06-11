#include "CalibrationTask.h"
#include "Notify.h"
#include "Input.h"

#include <Arduino.h>

namespace { // LOCAL SCOPE

int32_t tareRaw      = 0;
float   scaleFactor  = 1.0f;

} // anonymous namespace


namespace MetaSense::CalibrationTask { // EXTERNAL SCOPE

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
    // here we assume load cell raw is accessible via ADC directly
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

void resetAll()
{
    tareRaw     = 0;
    scaleFactor = 1.0f;
    MetaSense::Notify::sendStatus("Calibration reset");
}

} // namespace MetaSense::CalibrationTask
