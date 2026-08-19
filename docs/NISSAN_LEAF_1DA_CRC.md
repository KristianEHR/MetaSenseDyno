# Nissan Leaf 0x1DA CRC (Verified Algorithm)

This document captures the verified CRC algorithms used for CRC-bearing Nissan Leaf CAN frames in this codebase.

## Scope

- Frame: `0x1DA`
- Payload length: 8 bytes (`b0..b7`)
- Wire CRC byte: `b7`
- Clock selector: lower 2 bits of `b6` (`clock = b6 & 0x03`)

## Verified CRC

Compute CRC-8 MSB-first over 8 bytes:

1. Build synthetic input bytes: `[0xDA, b0, b1, b2, b3, b4, b5, b6]`
2. CRC parameters:
  - polynomial: `0x85`
  - init: `0x00`
  - xorOut: `0xBF`

Final calculated CRC is the direct CRC result.

## Acceptance Rule

A frame is CRC-valid when:

`b7 == calc`

No alternate/multi-method acceptance is used in the production path.

## Generated TX Frames

The active implementation uses verified CRC calculations for RX validation and TX generation.

The `0x1D4` and `0x11A` frames use the direct `0x1D` family:

- CRC-8 MSB-first
- polynomial: `0x1D`
- init: `0x00`
- xorOut: `0x29`
- input bytes: `[idLo, b0, b1, b2, b3, b4, b5, b6]`

## Reference Implementation

Current implementation locations:

- `src/CANBus.cpp`
  - `compute1daWireCrcFixed(...)`
  - `is1daWireCrcKnownGood(...)`
- `src/Input.cpp`
  - `computeLeaf1d4CrcConformant(...)`
  - `computeLeaf11aCrcConformant(...)`

## Notes

- Learner/debug logs (`[1DA-CRC-BAD]`) are diagnostic tools.
- Runtime acceptance uses the single global checker only.
- The verified `0x1DA` rule was fit against the captured 100-frame sample and cross-checked against earlier logs.
