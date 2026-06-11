#pragma once
#include <Arduino.h>

namespace MetaSense::CalibrationTask {

    void begin();   // optional init
    void loop();    // optional if you want async calibration

    // Commands from WebSocket
    void tare();
    void calibrate(float knownWeightKg);
    void setSingle(const String& key, const String& value);
    void resetAll();
    void resetToApMode();

}
