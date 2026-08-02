#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

struct LeafInvFeedback
{
    float rpm;            // Motor speed [rpm]
    float torque_nm;      // Actual torque [Nm]
    float input_voltage;  // MG input voltage [V] from 0x1DA
    float inverter_temp;  // Inverter IGBT temp [deg C]
    float stator_temp;    // Stator/winding temp [deg C]
    float coolant_temp;   // Coolant temp [deg C]

    bool ready;
    bool fault;
    bool warning;
    bool limp;

    uint32_t rpm_update_ms;
    uint32_t torque_update_ms;
    uint32_t torque_primary_update_ms;
    uint32_t torque_variant_update_ms;
    uint32_t temps_update_ms;
    uint32_t status_update_ms;
    uint32_t last_update_ms; // Last valid frame timestamp (system ms)

    // Per-ID decode counters for diagnostics.
    uint32_t rpm_frames;
    uint32_t torque_frames;
    uint32_t torque_primary_frames;
    uint32_t torque_variant_frames;
    uint32_t temps_frames;
    uint32_t temps_1da_frames;
    uint32_t temps_1dc_frames;
    uint32_t temps_55a_frames;
    uint32_t status_frames;

    // Explicit 0x55A DBC temperature channels (deg C).
    float id55a_motor_temp_c;
    float id55a_com_board_temp_c;
    float id55a_igbt_temp_c;
    float id55a_driver_board_temp_c;

    // Raw decode candidates for on-target frame mapping diagnostics.
    uint16_t rpm_raw01_le;
    uint16_t rpm_raw01_be;
    uint16_t rpm_raw23_le;
    uint16_t rpm_raw23_be;
    int16_t torque_raw01_le;
    int16_t torque_raw01_be;
    int16_t torque_raw23_le;
    int16_t torque_raw23_be;

    // ZE1 0x1DA aux status decode.
    uint8_t mg_clock;
    uint8_t mg_error_codes;
    uint8_t crc_1da;
    uint8_t id1da_raw[8]; // Raw 8-byte payload of last 0x1DA/0x01A frame
    float id1da_ze1_rpm;  // ZE1 Motorola-decoded RPM candidate
    float id1da_leg_rpm;  // Legacy LE word RPM candidate
    float id1da_ze1_tq;   // ZE1 Motorola-decoded torque candidate (Nm)
    float id1da_leg_tq;   // Legacy LE word torque candidate (Nm)

    // Thunderstruck TVCU 0x55B RX input decode.
    float id55b_torque_demand_nm;
    float id55b_like1da_rpm;
    float id55b_like1da_torque_nm;
    float id55b_ze1_rpm;  // ZE1 Motorola-decoded RPM candidate
    float id55b_leg_rpm;  // Legacy LE word RPM candidate
    float id55b_ze1_tq;   // ZE1 Motorola-decoded torque candidate (Nm)
    float id55b_leg_tq;   // Legacy LE word torque candidate (Nm)
    float id55b_like1da_input_voltage;
    uint32_t id55b_update_ms;
    uint32_t id55b_frames;
    uint32_t id55b_last_id;
    uint32_t id55b_primary_frames;
    uint32_t id05b_short_frames;
    uint32_t id50b_alias_frames;
    uint32_t id55b_primary_update_ms;
    uint8_t id55b_primary_raw[8];
    uint8_t id55b_raw[8];
    int16_t id55b_torque_raw_le;
    int16_t id55b_torque_raw_be;
    bool id55b_ready_cmd;
    bool id55b_hv_ok_cmd;
    bool id55b_gear_drive_cmd;
    uint8_t id55b_like1da_clock;
    uint8_t id55b_like1da_error_codes;
    uint8_t id55b_like1da_crc;
    uint8_t id55b_counter;
    uint8_t id55b_checksum;
    uint8_t id55b_checksum_calc;
    bool id55b_checksum_ok;
};

namespace LeafCan
{
    // Call this for every received TWAI frame.
    void decodeFrame(const twai_message_t &msg, LeafInvFeedback &fb, uint32_t now_ms);

    // Clear feedback to safe defaults.
    void reset(LeafInvFeedback &fb);
}
