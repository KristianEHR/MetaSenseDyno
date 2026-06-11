#pragma once
#include <Adafruit_NAU7802.h>

class LoadCellHAL {
public:
    bool begin();
    bool tare(uint16_t samples = 32);
    float readKg();
    bool isReady() const;

private:
    Adafruit_NAU7802 nau;
    bool ok = false;
};
