#pragma once
#include <Arduino.h>

struct Telemetry {
    float rpm;
    float hp;
    float torque;
    float brakeTorque;

    float peakHP;
    float peakHP_RPM;
    float peakTorque;
    float peakTorque_RPM;

    float leaf_rpm;
    float leaf_torqueNm;

    float leaf_invTempC;
    float leaf_motorTempC;
    float leaf_coolantTempC;

    bool  leaf_invReady;
    bool  leaf_invFault;
    bool  leaf_hvInterlockOk;
    uint8_t leaf_gearState;
};

extern Telemetry live;
