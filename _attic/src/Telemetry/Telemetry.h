#pragma once
#include <stdint.h>

namespace MetaSense {

class Telemetry {
public:
    // Core dyno telemetry (adapt as you like)
    float rpm    = 0.0f;
    float torque = 0.0f;
    float temp   = 0.0f;

    // Leaf inverter telemetry (used by LeafCan.cpp)
    float  leaf_rpm          = 0.0f;
    float  leaf_torqueNm     = 0.0f;
    float  leaf_invTempC     = 0.0f;
    float  leaf_motorTempC   = 0.0f;
    float  leaf_coolantTempC = 0.0f;

    bool   leaf_invReady      = false;
    bool   leaf_invFault      = false;
    bool   leaf_hvInterlockOk = false;
    uint8_t leaf_gearState    = 0;
};

} // namespace MetaSense
