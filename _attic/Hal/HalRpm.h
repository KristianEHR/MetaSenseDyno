#pragma once
#include <Arduino.h>

namespace HalRpm {

    void begin();

    float readEngineRpm();   // spark input
    float readDrumRpm();     // roller/drum hall input
}
