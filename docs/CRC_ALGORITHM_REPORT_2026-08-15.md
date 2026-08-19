# CRC Algorithm Report - 2026-08-15

## Summary

This report documents the current CRC algorithm implementation and field validation status for Leaf CAN integration.

Primary request addressed:

- Derive the exact `0x1DA` CRC rule from captured frames.
- Keep strict runtime handling for BAD `0x1DA` frames.

## Scope

Affected IDs and behavior:

- RX validation target: `0x1DA`
- Other Leaf family paths remain on the legacy shared candidate until separately revalidated.

Affected source files:

- `src/CANBus.cpp`
- `src/Input.cpp`
- `docs/NISSAN_LEAF_1DA_CRC.md`
- `README.md`

## Implemented Algorithm

The active `0x1DA` method is a direct CRC-8 calculation.

Algorithm:

1. Form byte vector: `[0xDA, b0, b1, b2, b3, b4, b5, b6]`
2. Compute CRC-8 MSB-first with:
   - poly: `0x85`
   - init: `0x00`
   - xorOut: `0xBF`
3. Final CRC:
   - `crc = crc8_msb(poly=0x85, init=0x00, xorOut=0xBF, bytes=[0xDA + b0..b6])`

## Runtime Acceptance Policy

`0x1DA` runtime policy:

- Per-frame check is mandatory.
- If CRC mismatches, frame is BAD and discarded (not used for decode/RPM updates).
- Fallback trust threshold is `10` consecutive BAD frames.

## Evidence Collected During Tuning

Captured logs showed recurring BAD lines with multiple `b0` values over time before the exact `0x1DA` rule was derived.

The verified rule matched the full 100-frame capture and the earlier short captures.

## Current Status

Build and deployment status:

- `Build (esp32s3-USB)`: PASS
- `Upload (esp32s3-USB)`: PASS

Code status:

- `0x1DA` now uses the verified direct CRC rule.
- Other family paths remain on the legacy shared candidate until separately revalidated.

## Risks and Next Validation Steps

Known risk:

- Other Leaf frame families may still need their own revalidation if they are expected to share the same helper.

Recommended validation loop:

1. Capture longer runtime windows under representative operating states.
2. Track `0x1DA` BAD-rate and consecutive BAD streaks.
3. If another family mismatch appears, fit that family independently.
4. Rebuild, reflash, and remeasure.

## Notes

This report documents the verified `0x1DA` implementation and field observations for that family.
