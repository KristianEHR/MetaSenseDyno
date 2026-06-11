#pragma once
#include <Arduino.h>

class RpmHAL {
public:
    bool begin(uint8_t pin);
    float getRPM();

private:
    static void IRAM_ATTR isr();
    static volatile uint32_t lastMicros;
    static volatile uint32_t period;
};
