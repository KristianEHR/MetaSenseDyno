#include "PwmHAL.h"

bool PwmHAL::begin(uint8_t pin, uint32_t freq, uint8_t channel) {
    _pin = pin;
    _channel = channel;
    ledcSetup(channel, freq, 10);
    ledcAttachPin(pin, channel);
    return true;
}

void PwmHAL::write(float duty) {
    duty = constrain(duty, 0.0f, 1.0f);
    ledcWrite(_channel, duty * 1023);
}
