#pragma once
#include <Arduino.h>

namespace HalLoadcell {

    void begin();

    void tare();
    float readRaw();   // raw ADC units
    float readKg();    // calibrated kg
}
