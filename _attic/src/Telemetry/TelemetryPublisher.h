#pragma once

#include <Arduino.h>
#include "Telemetry.h"

namespace TelemetryPublisher {

    // Initialize internal timers, etc.
    void begin();

    // Call from MetaSense::loopOnce()
    void loop();

    // Build JSON string from telemetry + recording flag
    String buildJson(const Telemetry &t, bool isRecording);

} // namespace TelemetryPublisher
