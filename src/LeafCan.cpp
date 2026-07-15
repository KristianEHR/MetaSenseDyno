#include "LeafCan.h"

#ifndef METASENSE_LEAF_VARIANT_120_55A
#define METASENSE_LEAF_VARIANT_120_55A 0
#endif

static inline float decodeMotorSpeed(const uint8_t *d)
{
    // BO_ 0x1DA MotorSpeed
    // SG_ MotorSpeed : 16|16@1+ (1,0) [0|20000] "rpm"
    const uint16_t raw = static_cast<uint16_t>(d[2]) |
                         (static_cast<uint16_t>(d[3]) << 8);
    return static_cast<float>(raw);
}

static inline float decodeMotorTorque(const uint8_t *d)
{
    // BO_ 0x1DB MotorTorque
    // SG_ MotorTorque : 16|16@1- (0.5,0) [-500|500] "Nm"
    const int16_t raw = static_cast<int16_t>(
        static_cast<uint16_t>(d[2]) | (static_cast<uint16_t>(d[3]) << 8));
    return static_cast<float>(raw) * 0.5f;
}

static inline void decodeTemps(const uint8_t *d,
                               float &inv,
                               float &stator,
                               float &coolant)
{
    // BO_ 0x1DC InverterTemps
    // SG_ InverterTemp : 0|8@1+ (1,-40) "C"
    // SG_ StatorTemp   : 8|8@1+ (1,-40) "C"
    // SG_ CoolantTemp  :16|8@1+ (1,-40) "C"
    inv = static_cast<float>(d[0]) - 40.0f;
    stator = static_cast<float>(d[1]) - 40.0f;
    coolant = static_cast<float>(d[2]) - 40.0f;
}

static inline void decodeTempsOffset1(const uint8_t *d,
                                      float &inv,
                                      float &stator,
                                      float &coolant)
{
    // Variant observed on this inverter: 0x55A packs three temp bytes at [1..3].
    inv = static_cast<float>(d[1]) - 40.0f;
    stator = static_cast<float>(d[2]) - 40.0f;
    coolant = static_cast<float>(d[3]) - 40.0f;
}

static inline void decodeStatus(const uint8_t *d,
                                bool &ready,
                                bool &fault,
                                bool &warning,
                                bool &limp)
{
    // BO_ 0x1D4 InverterStatus
    // SG_ Ready   : 0|1@1+
    // SG_ Fault   : 1|1@1+
    // SG_ Warning : 2|1@1+
    // SG_ LimpMode: 3|1@1+
    const uint8_t b = d[0];
    ready = (b & (1u << 0)) != 0;
    fault = (b & (1u << 1)) != 0;
    warning = (b & (1u << 2)) != 0;
    limp = (b & (1u << 3)) != 0;
}

void LeafCan::decodeFrame(const twai_message_t &msg,
                          LeafInvFeedback &fb,
                          uint32_t now_ms)
{
    const uint32_t id = msg.identifier;
    const uint8_t *d = msg.data;
    const uint8_t dlc = msg.data_length_code;

    switch (id)
    {
        case 0x1DA: // MotorSpeed (primary)
        case 0x01A: // MotorSpeed (short-ID variant)
            if (dlc >= 4U) {
                fb.rpm = decodeMotorSpeed(d);
                fb.rpm_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.rpm_frames;
            }
            break;

        case 0x1DB: // MotorTorque (primary)
        case 0x01B: // MotorTorque (short-ID variant)
            if (dlc >= 4U) {
                fb.torque_nm = decodeMotorTorque(d);
                fb.torque_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.torque_frames;
            }
            break;

        case 0x1DC: // InverterTemps (primary)
        case 0x01C: // InverterTemps (short-ID variant)
            if (dlc >= 3U) {
                decodeTemps(d, fb.inverter_temp, fb.stator_temp, fb.coolant_temp);
                fb.temps_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.temps_frames;
            }
            break;

        case 0x1D4: // InverterStatus (primary)
        case 0x014: // InverterStatus (short-ID variant)
            if (dlc >= 1U) {
                decodeStatus(d, fb.ready, fb.fault, fb.warning, fb.limp);
                fb.status_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.status_frames;
            }
            break;

#if METASENSE_LEAF_VARIANT_120_55A
        case 0x120: // Variant torque-like frame (bytes [2..3], big-endian, 0.1 Nm)
            if (dlc >= 4U) {
                const int16_t raw = static_cast<int16_t>(
                    (static_cast<uint16_t>(d[2]) << 8) | static_cast<uint16_t>(d[3]));
                fb.torque_nm = static_cast<float>(raw) * 0.1f;
                fb.torque_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.torque_frames;
            }
            break;

        case 0x55A: // Variant temps frame (bytes [1..3], +40 C offset)
            if (dlc >= 4U) {
                decodeTempsOffset1(d, fb.inverter_temp, fb.stator_temp, fb.coolant_temp);
                fb.temps_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.temps_frames;
            }
            break;
#endif

        default:
            // Ignore other IDs.
            break;
    }
}

void LeafCan::reset(LeafInvFeedback &fb)
{
    fb.rpm = 0.0f;
    fb.torque_nm = 0.0f;
    fb.inverter_temp = 0.0f;
    fb.stator_temp = 0.0f;
    fb.coolant_temp = 0.0f;

    fb.ready = false;
    fb.fault = false;
    fb.warning = false;
    fb.limp = false;

    fb.rpm_update_ms = 0;
    fb.torque_update_ms = 0;
    fb.temps_update_ms = 0;
    fb.status_update_ms = 0;
    fb.last_update_ms = 0;

    fb.rpm_frames = 0;
    fb.torque_frames = 0;
    fb.temps_frames = 0;
    fb.status_frames = 0;
}
