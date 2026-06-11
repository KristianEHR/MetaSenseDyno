#include "RpmHAL.h"

volatile uint32_t RpmHAL::lastMicros = 0;
volatile uint32_t RpmHAL::period = 0;

void IRAM_ATTR RpmHAL::isr() {
    uint32_t now = micros();
    period = now - lastMicros;
    lastMicros = now;
}

bool RpmHAL::begin(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(pin, isr, RISING);
    return true;
}

float RpmHAL::getRPM() {
    if (period == 0) return 0.0f;
    return 60.0f * (1e6f / period);
}
