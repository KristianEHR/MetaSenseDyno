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
    float throttlePercent = 0.0f;

    float torqueNm = 0.0f;
    float brakeTorqueNm = 0.0f;

    float energyMJ = 0.0f;

    float airDensity = 0.0f;
    float ambientC = 0.0f;
    float pressureHpa = 0.0f;

    float egtHotC = 0.0f;
    float egtAmbientC = 0.0f;
    uint8_t egtStatus = 0;
    int16_t egtAddress = -1;
    bool egtReady = false;
    int16_t egtAckAddress = -1;

    bool recording = false;

    float peakTorque = 0.0f;
    float peakTorque_RPM = 0.0f;
    float peakKW = 0.0f;
    float peakKW_RPM = 0.0f;

    float maxRpm = 0.0f;
    float maxTorqueNm = 0.0f;

    float rpmTarget = 0.0f;

    float kw = 0.0f;
    float humidity = 0.0f;
    float eTorque = 0.0f;
    float massflowM3h = 0.0f;
    float lambdaValue = 1.0f;
    float climateCF = 1.0f;   // climate correction factor (normalised to 1013.25 hPa / 20°C / 0% RH)
    DynoMode mode = DynoMode::Standard;

    bool vcuReady = false;  // GPIO 36: VCU / RB+ hardware interlock
    bool swActive = false;  // GPIO 35: panel SW switch (recording toggle)

    // Leaf inverter CAN telemetry (captured in parallel with analog/I2C sources).
    float leaf_rpm = 0.0f;
    float leaf_torqueNm = 0.0f;
    float leaf_invTempC = 0.0f;
    float leaf_statorTempC = 0.0f;
    float leaf_coolantTempC = 0.0f;
    bool leaf_invReady = false;
    bool leaf_invFault = false;
    bool leaf_invWarning = false;
    bool leaf_invLimp = false;
    uint32_t leaf_lastUpdateMs = 0;
};

} // namespace MetaSense
