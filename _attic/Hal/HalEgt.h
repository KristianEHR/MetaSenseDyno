#pragma once
#include <Arduino.h>

namespace HalEgt {

    void begin();

    float readHotC();
    float readAmbientC();
}
