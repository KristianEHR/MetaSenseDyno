#include "LeafCan.h"

#include <math.h>

#ifndef METASENSE_LEAF_VARIANT_120_55A
#define METASENSE_LEAF_VARIANT_120_55A 0
#endif

constexpr float kZe1TorqueBiasNm = -0.64f;

static inline float decodeMotorSpeed(const uint8_t *d)
{
    const uint16_t raw01 = static_cast<uint16_t>(d[0]) |
                           (static_cast<uint16_t>(d[1]) << 8);
    const uint16_t raw23 = static_cast<uint16_t>(d[2]) |
                           (static_cast<uint16_t>(d[3]) << 8);

    // Observed on target hardware: motor speed is in [2..3] little-endian.
    // Keep [0..1] little-endian as fallback for compatibility.
    return static_cast<float>((raw23 != 0U || raw01 == 0U) ? raw23 : raw01);
}

static inline float decodeInputVoltage(const uint8_t *d)
{
    // BO_ 0x1DA INVmc
    // SG_ MG_InputVoltage : 0|8@1+ (2,0) "V"
    return static_cast<float>(d[0]) * 2.0f;
}

static inline uint32_t extractMotorolaUnsigned(const uint8_t* d, int startBit, int length)
{
    if (length <= 0 || length > 32) {
        return 0U;
    }

    uint32_t value = 0U;
    int bit = startBit;
    for (int i = 0; i < length; ++i) {
        const int byteIndex = bit / 8;
        const int bitIndex = bit % 8;
        const uint32_t bitVal = (static_cast<uint32_t>(d[byteIndex]) >> bitIndex) & 0x1U;
        value = (value << 1) | bitVal;

        if ((bit % 8) == 0) {
            bit += 15;
        } else {
            --bit;
        }
    }
    return value;
}

static inline int32_t signExtend(uint32_t raw, int length)
{
    if (length <= 0 || length >= 32) {
        return static_cast<int32_t>(raw);
    }
    const uint32_t signBit = 1UL << (length - 1);
    if ((raw & signBit) == 0U) {
        return static_cast<int32_t>(raw);
    }
    const uint32_t mask = (1UL << length) - 1UL;
    return static_cast<int32_t>(raw | ~mask);
}

static inline float decodeZe1TorqueNm(const uint8_t* d)
{
    // SG_ MG_EffectiveTorque : 18|11@0- (0.5,0) "Nm"
    const uint32_t raw = extractMotorolaUnsigned(d, 18, 11);
    return (static_cast<float>(signExtend(raw, 11)) * 0.5f) + kZe1TorqueBiasNm;
}

static inline float decodeZe1OutputRevolution(const uint8_t* d)
{
    // SG_ MG_OutputRevolution : 39|15@0- (1,0) "rpm"
    const uint32_t raw = extractMotorolaUnsigned(d, 39, 15);
    return static_cast<float>(signExtend(raw, 15));
}

static inline uint8_t decodeZe1Clock(const uint8_t* d)
{
    // SG_ MG_CLOCK : 48|2@1+
    return static_cast<uint8_t>(d[6] & 0x03U);
}

static inline uint8_t decodeZe1ErrorCodes(const uint8_t* d)
{
    // SG_ MG_ErrorCodes : 50|6@1+
    return static_cast<uint8_t>((d[6] >> 2) & 0x3FU);
}

static inline uint8_t decodeZe1Crc(const uint8_t* d)
{
    // SG_ CRC_1DA : 56|8@1+
    return d[7];
}

static inline float decodeMotorTorque(const uint8_t *d)
{
    const int16_t raw01 = static_cast<int16_t>(
        static_cast<uint16_t>(d[0]) | (static_cast<uint16_t>(d[1]) << 8));
    const int16_t raw23 = static_cast<int16_t>(
        static_cast<uint16_t>(d[2]) | (static_cast<uint16_t>(d[3]) << 8));

    // Match RPM selection policy for this inverter family.
    const int16_t selectedRaw = (raw23 != 0 || raw01 == 0) ? raw23 : raw01;
    return static_cast<float>(selectedRaw) * 0.5f;
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
                fb.rpm_raw01_le = static_cast<uint16_t>(d[0]) |
                                  (static_cast<uint16_t>(d[1]) << 8);
                fb.rpm_raw01_be = (static_cast<uint16_t>(d[0]) << 8) |
                                  static_cast<uint16_t>(d[1]);
                fb.rpm_raw23_le = static_cast<uint16_t>(d[2]) |
                                  (static_cast<uint16_t>(d[3]) << 8);
                fb.rpm_raw23_be = (static_cast<uint16_t>(d[2]) << 8) |
                                  static_cast<uint16_t>(d[3]);
                fb.input_voltage = decodeInputVoltage(d);
                const float ze1Rpm = decodeZe1OutputRevolution(d);
                const float legacyRpm = decodeMotorSpeed(d);
                fb.rpm = (fabsf(ze1Rpm) <= 20000.0f) ? ze1Rpm : legacyRpm;

                if (dlc >= 8U) {
                    fb.mg_clock = decodeZe1Clock(d);
                    fb.mg_error_codes = decodeZe1ErrorCodes(d);
                    fb.crc_1da = decodeZe1Crc(d);

                    const float ze1Torque = decodeZe1TorqueNm(d);
                    const float legacyTorque = decodeMotorTorque(d);
                    fb.torque_nm = (fabsf(ze1Torque) <= 500.0f) ? ze1Torque : legacyTorque;
                    fb.torque_update_ms = now_ms;
                    fb.torque_primary_update_ms = now_ms;
                    ++fb.torque_frames;
                    ++fb.torque_primary_frames;
                }
                fb.rpm_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.rpm_frames;
            }
            break;

        case 0x1DB: // MotorTorque (primary)
        case 0x01B: // MotorTorque (short-ID variant)
            if (dlc >= 4U) {
                fb.torque_raw01_le = static_cast<int16_t>(
                    static_cast<uint16_t>(d[0]) | (static_cast<uint16_t>(d[1]) << 8));
                fb.torque_raw01_be = static_cast<int16_t>(
                    (static_cast<uint16_t>(d[0]) << 8) | static_cast<uint16_t>(d[1]));
                fb.torque_raw23_le = static_cast<int16_t>(
                    static_cast<uint16_t>(d[2]) | (static_cast<uint16_t>(d[3]) << 8));
                fb.torque_raw23_be = static_cast<int16_t>(
                    (static_cast<uint16_t>(d[2]) << 8) | static_cast<uint16_t>(d[3]));
                fb.torque_nm = decodeMotorTorque(d);
                fb.torque_update_ms = now_ms;
                fb.torque_primary_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.torque_frames;
                ++fb.torque_primary_frames;
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
    fb.input_voltage = 0.0f;
    fb.inverter_temp = 0.0f;
    fb.stator_temp = 0.0f;
    fb.coolant_temp = 0.0f;

    fb.ready = false;
    fb.fault = false;
    fb.warning = false;
    fb.limp = false;

    fb.rpm_update_ms = 0;
    fb.torque_update_ms = 0;
    fb.torque_primary_update_ms = 0;
    fb.torque_variant_update_ms = 0;
    fb.temps_update_ms = 0;
    fb.status_update_ms = 0;
    fb.last_update_ms = 0;

    fb.rpm_frames = 0;
    fb.torque_frames = 0;
    fb.torque_primary_frames = 0;
    fb.torque_variant_frames = 0;
    fb.temps_frames = 0;
    fb.status_frames = 0;

    fb.rpm_raw01_le = 0;
    fb.rpm_raw01_be = 0;
    fb.rpm_raw23_le = 0;
    fb.rpm_raw23_be = 0;
    fb.torque_raw01_le = 0;
    fb.torque_raw01_be = 0;
    fb.torque_raw23_le = 0;
    fb.torque_raw23_be = 0;
    fb.mg_clock = 0;
    fb.mg_error_codes = 0;
    fb.crc_1da = 0;
}
