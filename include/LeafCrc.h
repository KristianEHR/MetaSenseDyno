#pragma once

#include <stdint.h>
#include <string.h>

namespace MetaSense::LeafCRC {

// Source of truth for approved active Leaf CRC handling.
// Diagnostic or exploratory CRC candidates must never influence active verdicts
// or generated CRC bytes.

inline uint8_t crc8MsbPoly1DFF(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0xFFU;
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = static_cast<uint8_t>((crc << 1U) ^ 0x1DU);
            } else {
                crc = static_cast<uint8_t>(crc << 1U);
            }
        }
    }
    return static_cast<uint8_t>(crc ^ 0xFFU);
}

inline uint8_t crc8MsbPoly85BF(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00U;
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = static_cast<uint8_t>((crc << 1U) ^ 0x85U);
            } else {
                crc = static_cast<uint8_t>(crc << 1U);
            }
        }
    }
    return static_cast<uint8_t>(crc ^ 0xBFU);
}

inline uint8_t crc8MsbPoly1D29(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00U;
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = static_cast<uint8_t>((crc << 1U) ^ 0x1DU);
            } else {
                crc = static_cast<uint8_t>(crc << 1U);
            }
        }
    }
    return static_cast<uint8_t>(crc ^ 0x29U);
}

inline uint8_t computeBaseIdLo(uint8_t idLo, const uint8_t* payload8)
{
    if (payload8 == nullptr) {
        return 0U;
    }

    uint8_t p[8] = {0U};
    p[0] = idLo;
    memcpy(&p[1], payload8, 7U);
    return crc8MsbPoly1DFF(p, 8U);
}

inline uint8_t computeExact1daWireCrc(uint8_t idLo, const uint8_t* payload8)
{
    // TEST: Use Nissan standard 0x1D polynomial for 0x1DA RX (same as TX)
    // instead of the incorrect 0x85 polynomial that was causing CRC rejection
    if (payload8 == nullptr) {
        return 0U;
    }

    uint8_t p[8] = {0U};
    p[0] = idLo;
    memcpy(&p[1], payload8, 7U);
    // Use standard Nissan poly 0x1D with xorOut 0x29 instead of poly 0x85
    return crc8MsbPoly1D29(p, 8U);
}

inline uint8_t computeExact1d4LikeCrc(uint8_t idLo, const uint8_t* payload8)
{
    if (payload8 == nullptr) {
        return 0U;
    }

    uint8_t p[8] = {0U};
    p[0] = idLo;
    memcpy(&p[1], payload8, 7U);
    return crc8MsbPoly1D29(p, 8U);
}

inline uint8_t computeApprovedInverterCrc(uint8_t idLo, const uint8_t* payload8)
{
    if (payload8 == nullptr) {
        return 0U;
    }

    // Live 0x1D4 sniff fit on the current bus:
    // wire_crc = base_id_lo(poly=0x1D, init=0xFF, xorOut=0xFF over [idLo + b0..b6])
    //            XOR residue[HCM_CLOCK(byte4 bits6..7)].
    static constexpr uint8_t kClockResidue[4] = {0x6CU, 0xCBU, 0xA7U, 0x00U};
    const uint8_t base = computeBaseIdLo(idLo, payload8);
    // 0x1D4 clock is HCM_CLOCK at Intel bit 38 (byte 4 bits 6..7).
    // Other legacy paths keep the low-2-bits-in-byte-6 convention.
    const uint8_t clock = (idLo == 0xD4U)
        ? static_cast<uint8_t>((payload8[4] >> 6U) & 0x03U)
        : static_cast<uint8_t>(payload8[6] & 0x03U);
    return static_cast<uint8_t>(base ^ kClockResidue[clock]);
}

inline uint8_t computeSingleCandidate(uint8_t idLo, const uint8_t* payload8)
{
    // Compatibility alias: active callers should prefer computeApprovedInverterCrc().
    return computeApprovedInverterCrc(idLo, payload8);
}

} // namespace MetaSense::LeafCRC
