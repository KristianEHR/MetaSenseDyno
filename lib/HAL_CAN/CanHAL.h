#pragma once
#include <driver/twai.h>

class CanHAL {
public:
    bool begin(int txPin, int rxPin, twai_speed_t speed = TWAI_SPEED_500KBPS);
    bool send(uint32_t id, const uint8_t* data, uint8_t len);
    bool receive(uint32_t& id, uint8_t* data, uint8_t& len);
};
