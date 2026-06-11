#pragma once

#include <stdint.h>

namespace MetaSense {

enum class DynoMode : uint8_t {
    Standard,
    Brake,
    Inertia
};

inline const char* toString(DynoMode mode)
{
    switch (mode) {
    case DynoMode::Standard:
        return "STD";
    case DynoMode::Brake:
        return "brake";
    case DynoMode::Inertia:
        return "inertia";
    }

    return "STD";
}

struct Telemetry {
    float rpm = 0.0f;
    float drumRpm = 0.0f;
    float loadKg = 0.0f;

    float torqueNm = 0.0f;
    float brakeTorqueNm = 0.0f;

    float energyMJ = 0.0f;

    float airDensity = 0.0f;
    float ambientC = 0.0f;
    float pressureHpa = 0.0f;

    float egtHotC = 0.0f;
    float egtAmbientC = 0.0f;

    bool recording = false;

    float peakTorque = 0.0f;
    float peakTorque_RPM = 0.0f;

    float maxRpm = 0.0f;
    float maxTorqueNm = 0.0f;

    float rpmTarget = 0.0f;

    float kw = 0.0f;
    float humidity = 0.0f;
    float eTorque = 0.0f;
    DynoMode mode = DynoMode::Standard;
};

} // namespace MetaSense
