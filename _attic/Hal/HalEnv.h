#pragma once
#include <Arduino.h>

namespace HalEnv {

    void begin();
    void update();

    float airDensity();
    float ambientC();
    float pressureHpa();
}
