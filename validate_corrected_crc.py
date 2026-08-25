#!/usr/bin/env python3
"""
Final validation: Test the corrected 0x1D4 CRC algorithm (Poly 0x85, plain)
"""

def crc8_msb_poly85_plain(data):
    """Corrected 0x1D4 CRC: Polynomial 0x85, init 0x00, no xorOut"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x85) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def main():
    print("=== FINAL 0x1D4 CRC VALIDATION ===")
    print("Testing corrected algorithm: Poly 0x85 (plain), Payload only\n")
    
    # All 4 reference frames
    frames = [
        bytes.fromhex("6E 6E 00 00 87 44 01 23"),
        bytes.fromhex("6E 6E 00 00 C7 44 01 E4"),
        bytes.fromhex("6E 6E 00 00 07 44 01 28"),
        bytes.fromhex("6E 6E 00 00 47 44 01 EF"),
    ]
    
    expected_crcs = [0x23, 0xE4, 0x28, 0xEF]
    
    all_pass = True
    for i, (frame, expected) in enumerate(zip(frames, expected_crcs)):
        # CRC over 7-byte payload ONLY (no frame ID)
        payload = frame[:7]
        calculated = crc8_msb_poly85_plain(payload)
        passed = calculated == expected
        
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"Frame {i+1}: {status}")
        print(f"  Data:     {' '.join(f'{b:02X}' for b in frame[:7])}")
        print(f"  Expected: 0x{expected:02X}")
        print(f"  Calculated: 0x{calculated:02X}")
        
        if not passed:
            all_pass = False
        print()
    
    if all_pass:
        print("✓✓✓ ALL TESTS PASSED! ✓✓✓")
        print("\nCorrected Algorithm Summary:")
        print("  Polynomial: 0x85")
        print("  Initial value: 0x00")
        print("  XOR output: None (plain)")
        print("  Data: 7-byte payload ONLY (no frame ID)")
        print("\nFirmware update ready for deployment!")
    else:
        print("✗ Some tests failed")

if __name__ == "__main__":
    main()
