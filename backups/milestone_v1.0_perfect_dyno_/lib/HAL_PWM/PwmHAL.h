#pragma once
#include <Arduino.h>

class PwmHAL {
public:
    bool begin(uint8_t pin, uint32_t freq = 20000, uint8_t channel = 0);
    void write(float duty); // 0.0–1.0
private:
    uint8_t _pin;
    uint8_t _channel;
};
