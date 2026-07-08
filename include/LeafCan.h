#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

struct LeafInvFeedback
{
    float rpm;            // Motor speed [rpm]
    float torque_nm;      // Actual torque [Nm]
    float inverter_temp;  // Inverter IGBT temp [deg C]
    float stator_temp;    // Stator/winding temp [deg C]
    float coolant_temp;   // Coolant temp [deg C]

    bool ready;
    bool fault;
    bool warning;
    bool limp;

    uint32_t rpm_update_ms;
    uint32_t torque_update_ms;
    uint32_t temps_update_ms;
    uint32_t status_update_ms;
    uint32_t last_update_ms; // Last valid frame timestamp (system ms)
};

namespace LeafCan
{
    // Call this for every received TWAI frame.
    void decodeFrame(const twai_message_t &msg, LeafInvFeedback &fb, uint32_t now_ms);

    // Clear feedback to safe defaults.
    void reset(LeafInvFeedback &fb);
}
