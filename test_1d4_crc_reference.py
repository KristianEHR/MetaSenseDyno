#!/usr/bin/env python3
"""
0x1D4 CRC Validation Test
Tests our CRC algorithm against known-good reference frames from Thunderstruck TVCU
"""

def crc8_msb_poly_1d29(data):
    """CRC-8 MSB with polynomial 0x1D29 for 0x1D4 frames"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def compute_exact_1d4_wire_crc(payload8):
    """Wire CRC for 0x1D4 frame (ID + payload)"""
    frame = bytes([0xD4]) + payload8[:7]
    return crc8_msb_poly_1d29(frame)

def main():
    print("=== 0x1D4 CRC Reference Frame Validation ===")
    print("Known-good frames from Thunderstruck TVCU\n")
    
    # Reference frames with known-good CRC
    ref_frames = [
        {
            'data': bytes.fromhex("6E 6E 00 00 87 44 01 23"),
            'expected_crc': 0x23,
            'description': "Counter=8 (high), Charge=0x44, Byte5=0x87"
        },
        {
            'data': bytes.fromhex("6E 6E 00 00 C7 44 01 E4"),
            'expected_crc': 0xE4,
            'description': "Counter=C (high), Charge=0x44, Byte5=0xC7"
        },
        {
            'data': bytes.fromhex("6E 6E 00 00 07 44 01 28"),
            'expected_crc': 0x28,
            'description': "Counter=0 (high), Charge=0x44, Byte5=0x07"
        },
        {
            'data': bytes.fromhex("6E 6E 00 00 47 44 01 EF"),
            'expected_crc': 0xEF,
            'description': "Counter=4 (high), Charge=0x44, Byte5=0x47"
        },
    ]
    
    pass_count = 0
    fail_count = 0
    
    for idx, frame in enumerate(ref_frames):
        frame_data = frame['data']
        expected_crc = frame['expected_crc']
        
        # Calculate CRC using our algorithm
        calculated_crc = compute_exact_1d4_wire_crc(frame_data)
        
        # Display frame
        print(f"[Frame {idx+1}] {frame['description']}")
        print(f"  Data: {' '.join(f'{b:02X}' for b in frame_data)}")
        print(f"  Expected CRC:   0x{expected_crc:02X}")
        print(f"  Calculated CRC: 0x{calculated_crc:02X}")
        
        if calculated_crc == expected_crc:
            print("  ✓ PASS\n")
            pass_count += 1
        else:
            print("  ✗ FAIL - Mismatch!\n")
            fail_count += 1
    
    print("=== Summary ===")
    print(f"Passed: {pass_count}/{len(ref_frames)}")
    print(f"Failed: {fail_count}/{len(ref_frames)}")
    
    if fail_count == 0:
        print("\n✓ All tests PASSED! CRC algorithm is CORRECT.")
        print("\nKey findings:")
        print("  • Byte 5 low nibble = 0x7 (not 0x4!)")
        print("  • Byte 6 (charge status) = 0x44 (not 0x1E or 0x30!)")
        print("  • Counter in byte 5 high nibble cycles: 8→C→0→4")
        return 0
    else:
        print("\n✗ Tests FAILED! CRC algorithm needs adjustment.")
        return 1

if __name__ == "__main__":
    exit(main())
