#include "LoadCellHAL.h"

bool LoadCellHAL::begin() {
    if (!nau.begin()) return false;

    nau.setGain(NAU7802_GAIN_128);
    nau.setRate(NAU7802_RATE_80SPS);

    ok = true;
    return true;
}

bool LoadCellHAL::tare(uint16_t samples) {
    if (!ok) return false;
    return nau.calibrateZeroOffset(samples);
}

float LoadCellHAL::readKg() {
    if (!ok || !nau.available()) return 0.0f;
    return nau.read();
}

bool LoadCellHAL::isReady() const {
    return ok && nau.available();
}
