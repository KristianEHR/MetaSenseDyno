#include "LeafCan.h"

#include <Arduino.h>
#include <math.h>

#ifndef METASENSE_LEAF_VARIANT_120_55A
#define METASENSE_LEAF_VARIANT_120_55A 0
#endif

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

static inline float selectRpmCandidate(float ze1Rpm, float legacyRpm)
{
    const bool ze1Valid = isfinite(ze1Rpm) && (fabsf(ze1Rpm) <= 20000.0f);
    const bool legacyValid = isfinite(legacyRpm) && (fabsf(legacyRpm) <= 20000.0f);
    if (ze1Valid) {
        return ze1Rpm;
    }
    if (legacyValid) {
        return legacyRpm;
    }
    return 0.0f;
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
    return static_cast<float>(signExtend(raw, 11)) * 0.5f;
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
    const uint8_t b6 = d[6];
    const uint8_t stateNoClock = static_cast<uint8_t>(b6 & 0xFCU);

    // Observed benign Thunderstruck-driven states: 0x24 (+clock in bits 0..1)
    // and occasionally 0x18 (+clock). These should not be shown as active errors.
    if (stateNoClock == 0x24U || stateNoClock == 0x18U) {
        return 0U;
    }

    return static_cast<uint8_t>((b6 >> 2) & 0x3FU);
}

static inline uint8_t decodeZe1Crc(const uint8_t* d)
{
    // SG_ CRC_1DA : 56|8@1+
    return d[7];
}

static inline uint8_t decodeInvStatusBit(const uint8_t* d)
{
    // SG_ Inv_StatusBit : 40|1@1+
    return static_cast<uint8_t>((d[5] >> 0U) & 0x01U);
}

static inline uint8_t decodeInvFaultMap(const uint8_t* d)
{
    // SG_ Inv_FaultMap : 50|6@1+
    return static_cast<uint8_t>((d[6] >> 2U) & 0x3FU);
}

static inline uint8_t decodeInvBlinky(const uint8_t* d)
{
    // SG_ Inv_Blinky : 14|2@1+
    return static_cast<uint8_t>((d[1] >> 6U) & 0x03U);
}

static inline uint16_t decodeInvUnknownFaults(const uint8_t* d)
{
    // SG_ Inv_UnknownFaults : 13|11@0+
    return static_cast<uint16_t>(extractMotorolaUnsigned(d, 13, 11));
}

static inline uint8_t decodeInvFaultCanTimeoutMaybe(const uint8_t* d)
{
    // SG_ Inv_Fault_CANTimeoutMaybe : 23|1@1+
    return static_cast<uint8_t>((d[2] >> 7U) & 0x01U);
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

static inline void decodeTemps55aDbc(const uint8_t* d,
                                     float& motorTempC,
                                     float& comBoardTempC,
                                     float& igbtTempC,
                                     float& driverBoardTempC)
{
    // DBC: BO_ 0x55A INVmc
    // SG_ MotorTemperature            : 40|8  (unit "dC*2")
    // SG_ InverterComBoardTemp        : 8|8   (unit "dC*2")
    // SG_ IGBTTemperature             : 16|8  (unit "dC*2")
    // SG_ IGBTDriverBoardTemperature  : 24|8  (unit "dC*2")
    // Variant calibration on this inverter: 0x55A temperature bytes behave as raw-40.
    motorTempC = static_cast<float>(d[5]) - 40.0f;
    comBoardTempC = static_cast<float>(d[1]) - 40.0f;
    igbtTempC = static_cast<float>(d[2]) - 40.0f;
    driverBoardTempC = static_cast<float>(d[3]) - 40.0f;
}

static inline bool decodeTempsFrom1daFallback(const uint8_t* d,
                                              uint8_t dlc,
                                              float& inv,
                                              float& stator,
                                              float& coolant)
{
    if (dlc < 7U) {
        return false;
    }

    // Observed fallback mapping on some inverter variants when 0x1DC is absent.
    // Keep guarded by plausibility checks to avoid publishing obvious garbage.
    const float candInv = static_cast<float>(d[4]) - 40.0f;
    const float candStator = static_cast<float>(d[5]) - 40.0f;
    const float candCoolant = static_cast<float>(d[6]) - 40.0f;

    const bool plausible = (candInv > -35.0f && candInv < 180.0f) &&
                           (candStator > -35.0f && candStator < 180.0f) &&
                           (candCoolant > -35.0f && candCoolant < 180.0f);
    if (!plausible) {
        return false;
    }

    inv = candInv;
    stator = candStator;
    coolant = candCoolant;
    return true;
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
    // Some target variants expose the same flags with the low nibble shifted or
    // inverted. Treat both common layouts as valid so the monitor can still show
    // a meaningful READY/fault state even when the raw bit layout differs.
    // For this low-HV test setup, a zeroed status byte (0x00) and the observed
    // benign state bytes 0x18/0x24 are treated as OK, not as warning/fault.
    const uint8_t b = d[0];
    const bool benignNoHvStatus = (b == 0x00U) || (b == 0x18U) || (b == 0x24U);

    if (benignNoHvStatus) {
        ready = true;
        fault = false;
        warning = false;
        limp = false;
        return;
    }

    const bool directReady = (b & (1u << 0)) != 0;
    const bool directFault = (b & (1u << 1)) != 0;
    const bool directWarning = (b & (1u << 2)) != 0;
    const bool directLimp = (b & (1u << 3)) != 0;

    const bool shiftedReady = (b & (1u << 4)) != 0;
    const bool shiftedFault = (b & (1u << 5)) != 0;
    const bool shiftedWarning = (b & (1u << 6)) != 0;
    const bool shiftedLimp = (b & (1u << 7)) != 0;

    ready = directReady || shiftedReady;
    fault = directFault || shiftedFault;
    warning = directWarning || shiftedWarning;
    limp = directLimp || shiftedLimp;
}

static inline void decode1daStatusBits(const uint8_t* d,
                                        uint8_t dlc,
                                        uint8_t& statusByte,
                                        uint8_t& statusBits,
                                        bool& ready,
                                        bool& fault,
                                        bool& warning,
                                        bool& limp)
{
    statusByte = 0U;
    statusBits = 0U;
    ready = false;
    fault = false;
    warning = false;
    limp = false;

#if METASENSE_LEAF_1DA_SNIFF_DECODE
    if (dlc == 0U) {
        return;
    }

    const uint8_t byteCount = (dlc < 8U) ? dlc : 8U;
    for (uint8_t byteIdx = 0U; byteIdx < byteCount; ++byteIdx) {
        const uint8_t rawByte = d[byteIdx];
        const uint8_t lowNibble = static_cast<uint8_t>(rawByte & 0x0FU);
        const uint8_t highNibble = static_cast<uint8_t>((rawByte >> 4U) & 0x0FU);

        const uint8_t candidateBytes[2] = {lowNibble, highNibble};
        for (uint8_t candidateIdx = 0U; candidateIdx < 2U; ++candidateIdx) {
            const uint8_t candidateNibble = candidateBytes[candidateIdx];
            const bool candidateIsBenign = (candidateNibble == 0x00U) || (candidateNibble == 0x08U) || (candidateNibble == 0x04U) || (candidateNibble == 0x0CU);
            if (candidateNibble == 0U && byteIdx >= 2U) {
                continue;
            }
            if (candidateIsBenign && candidateNibble == 0U) {
                continue;
            }

            statusByte = rawByte;
            statusBits = candidateNibble;
            ready = (statusBits & 0x01U) != 0U;
            fault = (statusBits & 0x02U) != 0U;
            warning = (statusBits & 0x04U) != 0U;
            limp = (statusBits & 0x08U) != 0U;
            if (statusBits != 0U) {
                return;
            }
        }
    }

    // Fallback to the previous byte-6 assumption when nothing else looks useful.
    statusByte = (byteCount > 6U) ? d[6] : d[0];
    statusBits = static_cast<uint8_t>(statusByte & 0x0FU);
    ready = (statusBits & 0x01U) != 0U;
    fault = (statusBits & 0x02U) != 0U;
    warning = (statusBits & 0x04U) != 0U;
    limp = (statusBits & 0x08U) != 0U;
#endif
}

static inline void decode1daLikeFrame(const uint8_t* d,
                                      uint8_t dlc,
                                      uint16_t& raw01Le,
                                      uint16_t& raw01Be,
                                      uint16_t& raw23Le,
                                      uint16_t& raw23Be,
                                      float& inputVoltage,
                                      float& rpm,
                                      float& torqueNm,
                                      uint8_t& clock,
                                      uint8_t& errorCodes,
                                      uint8_t& crc,
                                      bool& hasTorque)
{
    raw01Le = 0U;
    raw01Be = 0U;
    raw23Le = 0U;
    raw23Be = 0U;
    inputVoltage = 0.0f;
    rpm = 0.0f;
    torqueNm = 0.0f;
    clock = 0U;
    errorCodes = 0U;
    crc = 0U;
    hasTorque = false;

    if (dlc < 4U) {
        return;
    }

    raw01Le = static_cast<uint16_t>(d[0]) |
              (static_cast<uint16_t>(d[1]) << 8);
    raw01Be = (static_cast<uint16_t>(d[0]) << 8) |
              static_cast<uint16_t>(d[1]);
    raw23Le = static_cast<uint16_t>(d[2]) |
              (static_cast<uint16_t>(d[3]) << 8);
    raw23Be = (static_cast<uint16_t>(d[2]) << 8) |
              static_cast<uint16_t>(d[3]);
    inputVoltage = decodeInputVoltage(d);

    const float ze1Rpm = decodeZe1OutputRevolution(d);
    const float legacyRpm = decodeMotorSpeed(d);
    rpm = selectRpmCandidate(ze1Rpm, legacyRpm);

    if (dlc < 8U) {
        return;
    }

    clock = decodeZe1Clock(d);
    errorCodes = decodeZe1ErrorCodes(d);
    crc = decodeZe1Crc(d);

    const float ze1Torque = decodeZe1TorqueNm(d);
    const float legacyTorque = decodeMotorTorque(d);
    torqueNm = (fabsf(ze1Torque) <= 500.0f) ? ze1Torque : legacyTorque;
    hasTorque = true;
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
        case 0x1DA: // MotorSpeed (accepted)
            if (dlc >= 4U) {
                // Capture raw payload bytes for diagnostics.
                for (uint8_t i = 0U; i < 8U; ++i) {
                    fb.id1da_raw[i] = (i < dlc) ? d[i] : 0U;
                }
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
                fb.rpm = selectRpmCandidate(ze1Rpm, legacyRpm);
                fb.id1da_ze1_rpm = ze1Rpm;
                fb.id1da_leg_rpm = legacyRpm;

                fb.id1da_ze1_tq = 0.0f;
                fb.id1da_leg_tq = 0.0f;
                if (dlc >= 8U) {
                    fb.mg_clock = decodeZe1Clock(d);
                    fb.mg_error_codes = decodeZe1ErrorCodes(d);
                    fb.crc_1da = decodeZe1Crc(d);
                    fb.inv_status_bit = decodeInvStatusBit(d);
                    fb.inv_fault_map = decodeInvFaultMap(d);
                    fb.inv_blinky = decodeInvBlinky(d);
                    fb.inv_unknown_faults = decodeInvUnknownFaults(d);
                    fb.inv_fault_can_timeout_maybe = decodeInvFaultCanTimeoutMaybe(d);

                    const float ze1Torque = decodeZe1TorqueNm(d);
                    const float legacyTorque = decodeMotorTorque(d);
                    fb.torque_nm = (fabsf(ze1Torque) <= 500.0f) ? ze1Torque : legacyTorque;
                    fb.id1da_ze1_tq = ze1Torque;
                    fb.id1da_leg_tq = legacyTorque;
                    fb.torque_update_ms = now_ms;
                    fb.torque_primary_update_ms = now_ms;
                    ++fb.torque_frames;
                    ++fb.torque_primary_frames;
                }

                // Benign startup/no-HV states on this inverter family can contain
                // residual non-zero raw bits that look like valid RPM in the raw DBC
                // fields. Treat those as zero until the inverter reports an active
                // ready state and a normal live RPM stream.
                const uint8_t statusByteZeroMask = (dlc > 0U) ? d[0] : 0U;
                const uint8_t startupMask = 0x00U;
                const bool startupStyleStatus = (statusByteZeroMask == 0x00U) ||
                                               (statusByteZeroMask == 0x18U) ||
                                               (statusByteZeroMask == 0x24U);
                if (startupStyleStatus || (fb.id1da_status_bits == startupMask)) {
                    fb.rpm = 0.0f;
                    fb.torque_nm = 0.0f;
                }

#if METASENSE_LEAF_1DA_SNIFF_DECODE
                if (dlc >= 1U) {
                    uint8_t statusByte = 0U;
                    uint8_t statusBits = 0U;
                    bool statusReady = false;
                    bool statusFault = false;
                    bool statusWarning = false;
                    bool statusLimp = false;
                    decode1daStatusBits(d, dlc, statusByte, statusBits, statusReady, statusFault, statusWarning, statusLimp);
                    fb.id1da_status_byte = statusByte;
                    fb.id1da_status_bits = statusBits;
                    fb.id1da_status_ready = statusReady;
                    fb.id1da_status_fault = statusFault;
                    fb.id1da_status_warning = statusWarning;
                    fb.id1da_status_limp = statusLimp;
                    Serial.printf("[1DA-SNIFF] len=%u selected_byte=0x%02X status_bits=0x%02X ready=%u fault=%u warning=%u limp=%u\n",
                                  static_cast<unsigned>(dlc),
                                  static_cast<unsigned>(statusByte),
                                  static_cast<unsigned>(statusBits),
                                  statusReady ? 1U : 0U,
                                  statusFault ? 1U : 0U,
                                  statusWarning ? 1U : 0U,
                                  statusLimp ? 1U : 0U);
                }
#endif

                float invFallback = 0.0f;
                float statorFallback = 0.0f;
                float coolantFallback = 0.0f;
                if (decodeTempsFrom1daFallback(d, dlc, invFallback, statorFallback, coolantFallback)) {
                    fb.inverter_temp = invFallback;
                    fb.stator_temp = statorFallback;
                    fb.coolant_temp = coolantFallback;
                    fb.temps_update_ms = now_ms;
                    ++fb.temps_frames;
                    ++fb.temps_1da_frames;
                }

                fb.rpm_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.rpm_frames;
            }
            break;

        case 0x1DB: // MotorTorque (accepted)
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

        case 0x1DC: // InverterTemps (accepted)
            if (dlc >= 3U) {
                decodeTemps(d, fb.inverter_temp, fb.stator_temp, fb.coolant_temp);
                fb.temps_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.temps_frames;
                ++fb.temps_1dc_frames;
            }
            break;

        case 0x55A: // Alternate temps frame (DBC-provided)
            if (dlc >= 6U) {
                float motorTempC = 0.0f;
                float comBoardTempC = 0.0f;
                float igbtTempC = 0.0f;
                float driverBoardTempC = 0.0f;
                decodeTemps55aDbc(d,
                                  motorTempC,
                                  comBoardTempC,
                                  igbtTempC,
                                  driverBoardTempC);

                fb.id55a_motor_temp_c = motorTempC;
                fb.id55a_com_board_temp_c = comBoardTempC;
                fb.id55a_igbt_temp_c = igbtTempC;
                fb.id55a_driver_board_temp_c = driverBoardTempC;

                // Maintain app's generic temperature channels from 0x55A.
                const bool motorMissing = (motorTempC <= 0.1f) &&
                                          (driverBoardTempC > 0.1f || comBoardTempC > 0.1f || igbtTempC > 0.1f);
                const float motorEffectiveC = motorMissing ? driverBoardTempC : motorTempC;
                fb.inverter_temp = igbtTempC;
                fb.stator_temp = motorEffectiveC;
                fb.coolant_temp = comBoardTempC;
                fb.temps_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.temps_frames;
                ++fb.temps_55a_frames;
            }
            break;

        case 0x1D4: // Inverter status
            if (dlc >= 1U) {
                decodeStatus(d, fb.ready, fb.fault, fb.warning, fb.limp);
                fb.status_update_ms = now_ms;
                fb.last_update_ms = now_ms;
                ++fb.status_frames;
            }
            break;


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
    fb.temps_1da_frames = 0;
    fb.temps_1dc_frames = 0;
    fb.temps_55a_frames = 0;
    fb.status_frames = 0;

    fb.id55a_motor_temp_c = 0.0f;
    fb.id55a_com_board_temp_c = 0.0f;
    fb.id55a_igbt_temp_c = 0.0f;
    fb.id55a_driver_board_temp_c = 0.0f;

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
    fb.inv_status_bit = 0;
    fb.inv_fault_map = 0;
    fb.inv_blinky = 0;
    fb.inv_unknown_faults = 0;
    fb.inv_fault_can_timeout_maybe = 0;
    fb.id1da_status_byte = 0;
    fb.id1da_status_bits = 0;
    fb.id1da_status_ready = false;
    fb.id1da_status_fault = false;
    fb.id1da_status_warning = false;
    fb.id1da_status_limp = false;
    for (uint8_t i = 0U; i < 8U; ++i) fb.id1da_raw[i] = 0U;
    fb.id1da_ze1_rpm = 0.0f;
    fb.id1da_leg_rpm = 0.0f;
    fb.id1da_ze1_tq = 0.0f;
    fb.id1da_leg_tq = 0.0f;
}
