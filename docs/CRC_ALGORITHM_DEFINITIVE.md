# Nissan Leaf CAN Frame CRC Algorithm - Definitive Reference

**Status**: ✅ VALIDATED AND WORKING  
**Last Update**: August 25, 2026  
**Validated Against**: 4 known-good Thunderstruck TVCU reference frames

---

## Executive Summary

This document defines the **ONLY correct CRC algorithms** for Nissan Leaf CAN frame generation and validation. These algorithms have been reverse-engineered through iterative frame analysis and validated against factory-captured frames from a Thunderstruck VCU operating with the Nissan inverter.

### Key Discovery

**0x1D4 and 0x1DA use the same polynomial (0x85)**, but differ in:
- Frame ID inclusion
- XOR output application
- Input length

---

## CRC Algorithm Specification

### Frame 0x1DA (RX Validation from Inverter)

**Purpose**: Validate inverter status frames received on CAN bus.

**Algorithm**:
- **Polynomial**: 0x85 (MSB-first)
- **Initial value**: 0x00
- **XOR output**: 0xBF
- **Input**: Frame ID (0xDA) + payload bytes 0-6 (7 bytes total)

**Pseudocode**:
```
crc = 0x00
for each byte in [0xDA, b0, b1, b2, b3, b4, b5, b6]:
    crc ^= byte
    for bit in 0..7:
        if crc & 0x80:
            crc = (crc << 1) ^ 0x85
        else:
            crc = crc << 1
crc = crc ^ 0xBF
return crc
```

**C++ Implementation**:
```cpp
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

inline uint8_t computeExact1daWireCrc(uint8_t idLo, const uint8_t* payload8)
{
    uint8_t p[8] = {0U};
    p[0] = idLo;  // 0xDA
    memcpy(&p[1], payload8, 7U);
    return crc8MsbPoly85BF(p, 8U);
}
```

**Validation Rule**:
```
if (receivedCrc == computeExact1daWireCrc(0xDA, payload)) {
    // Frame accepted
} else {
    // Frame rejected
}
```

---

### Frame 0x1D4 (TX Generation to Inverter)

**Purpose**: Generate torque demand frames sent from VCU to inverter.

**Algorithm**:
- **Polynomial**: 0x85 (MSB-first) — **SAME as 0x1DA**
- **Initial value**: 0x00
- **XOR output**: None (plain output)
- **Input**: **Payload bytes 0-6 only** (7 bytes, NO frame ID prepended)

**Critical Difference from 0x1DA**:
- Frame ID (0xD4) is **NOT** prepended to input
- XOR output (0xBF) is **NOT** applied
- Only payload is used

**Pseudocode**:
```
crc = 0x00
for each byte in [b0, b1, b2, b3, b4, b5, b6]:  // NO frame ID
    crc ^= byte
    for bit in 0..7:
        if crc & 0x80:
            crc = (crc << 1) ^ 0x85
        else:
            crc = crc << 1
// NO XOR output
return crc
```

**C++ Implementation**:
```cpp
inline uint8_t crc8MsbPoly85Plain(const uint8_t* data, uint8_t len)
{
    // 0x1D4 TX CRC: Polynomial 0x85, init 0x00, no xorOut
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
    return crc;  // No XOR output - critical difference!
}

inline uint8_t computeExact1d4LikeCrc(uint8_t idLo, const uint8_t* payload8)
{
    // 0x1D4 TX CRC: Polynomial 0x85, init 0x00, no xorOut, NO frame ID prepended
    // Only payload bytes 0-6 used (byte 7 is CRC field itself)
    (void)idLo;  // Frame ID NOT used for 0x1D4 CRC
    return crc8MsbPoly85Plain(payload8, 7U);
}
```

**Usage in Frame Construction**:
```cpp
uint8_t payload[8] = {
    0x6E,  // b0: static
    0x6E,  // b1: static
    0x00,  // b2: static
    0x00,  // b3: static
    0x87,  // b4: rolling counter (cycles 0x87→0xC7→0x07→0x47)
    0x44,  // b5: static charge status
    0x01,  // b6: static field
    0x00   // b7: will hold CRC
};

payload[7] = computeExact1d4LikeCrc(0xD4, payload);  // Compute CRC
canSend(0x1D4, payload, 8);
```

---

### Frame 0x11A (TX Echo/Keep-Alive)

**Purpose**: Multiplexed heartbeat frame with per-mux variant payloads.

**CRC Algorithm**: None - **0x11A has no CRC field**

**Frame Structure**:
```
Byte 0: 0x4E (static)
Byte 1: 0x40 (static) — corrected from erroneous 0xA0
Byte 2: 0x00 (static)
Byte 3: 0xAA (static)
Byte 4: 0xC0 (static)
Byte 5: 0x00 (static)
Byte 6: Mux selector [0, 1, 2, 3] — cycled each transmission
Byte 7: Mux-dependent value
        - Mux 0: 0x6B
        - Mux 1: 0xEE
        - Mux 2: 0xE4
        - Mux 3: 0x61
```

**Transmission Pattern**:
```
Frame N:   0x4E 0x40 0x00 0xAA 0xC0 0x00 0x00 0x6B  (mux=0)
Frame N+1: 0x4E 0x40 0x00 0xAA 0xC0 0x00 0x01 0xEE  (mux=1)
Frame N+2: 0x4E 0x40 0x00 0xAA 0xC0 0x00 0x02 0xE4  (mux=2)
Frame N+3: 0x4E 0x40 0x00 0xAA 0xC0 0x00 0x03 0x61  (mux=3)
Frame N+4: 0x4E 0x40 0x00 0xAA 0xC0 0x00 0x00 0x6B  (mux=0, repeat)
```

---

## Validation Testing

### Reference Frames (Known Good - Thunderstruck TVCU)

These frames were captured while the inverter was actively accepting torque commands:

```
Frame 1: 6E 6E 00 00 87 44 01 23  (counter=0, CRC=0x23)
Frame 2: 6E 6E 00 00 C7 44 01 E4  (counter=1, CRC=0xE4)
Frame 3: 6E 6E 00 00 07 44 01 28  (counter=2, CRC=0x28)
Frame 4: 6E 6E 00 00 47 44 01 EF  (counter=3, CRC=0xEF)
```

### Validation Matrix

| Frame | Input (b0..b6) | Algorithm | Expected CRC | Status |
|-------|---|---|---|---|
| 1 | 6E 6E 00 00 87 44 01 | crc8MsbPoly85Plain | 0x23 | ✅ PASS |
| 2 | 6E 6E 00 00 C7 44 01 | crc8MsbPoly85Plain | 0xE4 | ✅ PASS |
| 3 | 6E 6E 00 00 07 44 01 | crc8MsbPoly85Plain | 0x28 | ✅ PASS |
| 4 | 6E 6E 00 00 47 44 01 | crc8MsbPoly85Plain | 0xEF | ✅ PASS |

---

## Historical Investigation Notes

### Wrong Approaches (Reference Only)

**1. Polynomial 0x1D29 with Init 0x00, XorOut 0x29** ❌
- Failed all 4 reference frames
- Resulted in inverter status "100 000" (frame rejection)
- Source: Initial reverse-engineering attempt
- **DO NOT USE**

**2. Polynomial 0x1D with Init 0xFF, XorOut 0xFF** ❌
- Approximation algorithm with clock residue table
- Failed to match any known-good frames
- Source: Complexity bias (thought special handling was needed)
- **DO NOT USE**

**3. Polynomial 0x85 WITH Frame ID (0xD4) Prepended** ❌
- Used the 0x1DA algorithm structure for 0x1D4
- Failed all 4 reference frames
- Inverter rejected frames with status "100 000"
- **DO NOT USE**

### Correct Discovery Process

1. **Week 1-2**: Tried polynomial 0x1D29, 0x1D, various init/xorOut combinations
   - Result: All failed, inverter status "100 000"
   
2. **Week 3**: Analyzed DBC signal definitions
   - Found: 0x1D4 uses `HCM_CLOCK` at bit 38 (byte 4 bits 6-7)
   - Hypothesized: Counter encoding non-linear (lookup table)
   - Result: Frame structure correct, CRC still wrong

3. **Week 4**: Examined 0x1DA algorithm structure
   - Observed: Both 0x1DA and 0x1D4 reject with status "100 000"
   - Hypothesis: Share same base polynomial (0x85)
   
4. **Week 5**: Tested polynomial 0x85 on 0x1D4 payload
   - **Critical insight**: Frame ID NOT prepended for 0x1D4
   - **Result**: First frame matched (CRC=0x23)
   - **Validation**: Remaining 3 frames matched perfectly
   - **Status**: ✅ SUCCESS

---

## Implementation Checklist

- [x] Polynomial 0x85 MSB-first algorithm implemented
- [x] Frame ID (0xDA) prepended for 0x1DA validation
- [x] Frame ID (0xD4) **NOT** prepended for 0x1D4 generation
- [x] XOR output (0xBF) applied for 0x1DA, NOT applied for 0x1D4
- [x] Rolling counter (0x87→0xC7→0x07→0x47) verified
- [x] 0x11A static bytes and mux values correct
- [x] Runtime validation: reject BAD 0x1DA frames
- [x] Runtime monitoring: report CRC errors in telemetry
- [x] All 4 reference frames validated ✅

---

## Active Runtime Policy

### RX Validation (0x1DA)

```cpp
if (computeExact1daWireCrc(0xDA, rxPayload) != rxPayload[7]) {
    ++badFrameCount;
    if (badFrameCount >= 10) {
        disableCan();  // Disable CAN fallback after 10 consecutive bad frames
    }
    return;  // Frame rejected
}
```

### TX Generation (0x1D4)

```cpp
// Every 10ms (leafTxPacerTask):
s_leaf1d4PayloadCached[4] = (COUNTER_NIBBLE_MAP[hcmClock] << 4) | 0x7;
s_leaf1d4PayloadCached[7] = computeExact1d4LikeCrc(0xD4, s_leaf1d4PayloadCached);
canSend(0x1D4, s_leaf1d4PayloadCached, 8);
```

### TX Keep-Alive (0x11A)

```cpp
// Every 10ms (sendLeafKeepAlive11a):
muxSelector = (muxSelector + 1) & 0x03;
payload[6] = muxSelector;
payload[7] = muxTemplateB7[muxSelector];  // No CRC needed
canSend(0x11A, payload, 8);
```

---

## Error Indicators

**Inverter Status "100 000"** = Frames being rejected
- Possible causes:
  - 0x1D4 CRC incorrect
  - 0x1D4 counter not cycling (must go 0x87→0xC7→0x07→0x47 every 10ms)
  - 0x11A not transmitted or mux not cycling
  - 0x11A byte 1 incorrect (must be 0x40, not 0xA0)

**Inverter Status "000 000"** = Frames accepted ✅

---

## References

- **CRC-Rules.txt**: Nissan Leaf protocol specification archive
- **Leaf-CAN_ZE1.dbc**: CAN database with signal definitions
- **include/LeafCrc.h**: Active implementation (C++ functions)
- **src/Input.cpp**: Frame generation and telemetry pipeline

---

## Maintenance Notes

If future debugging becomes necessary:

1. **Always validate against known-good reference frames**
   - These 4 frames (Frame 1-4 above) are the ground truth
   - Never hypothesize; test against these frames first

2. **The polynomial 0x85 is definitive**
   - Nissan Leaf uses only two CRC polynomials in the DBC:
     - 0x85 (0x1DA RX, 0x1D4 TX)
     - 0x1D with clock residue (legacy, rarely used)

3. **Frame ID inclusion is frame-specific**
   - 0x1DA: prepend 0xDA (RX validation)
   - 0x1D4: NO prepend (TX generation only)

4. **Never add complexity layers without frame-level validation**
   - Clock residue tables, polynomial approximations, etc. are attractive but fail
   - Always verify against captured wire frames first

---

**End of Document**  
*Validated and working as of August 25, 2026*
