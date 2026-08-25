#!/usr/bin/env python3
"""
0x1D4 CRC Algorithm Discovery
Test multiple CRC approaches to find which matches the known-good frames
"""

def crc8_msb(data, poly):
    """Generic CRC-8 MSB with variable polynomial"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def test_algorithm(name, poly_low8, payload, expected_crc, include_id=True, crc_included=False):
    """Test a CRC algorithm variant"""
    if include_id:
        frame = bytes([0xD4]) + payload[:7]
    else:
        frame = payload[:7]
    
    if crc_included:
        # Include the CRC byte in calculation (usually 0x00 or the expected value)
        frame = frame + bytes([0x00])
    
    calculated = crc8_msb(frame, poly_low8)
    status = "✓ PASS" if calculated == expected_crc else "✗ FAIL"
    return calculated == expected_crc, calculated, status

def main():
    print("=== 0x1D4 CRC Algorithm Discovery ===\n")
    
    # Known reference frame
    frame_data = bytes.fromhex("6E 6E 00 00 C7 44 01 E4")
    payload = frame_data[:7]
    expected_crc = 0xE4
    
    print(f"Test Frame: {' '.join(f'{b:02X}' for b in frame_data)}")
    print(f"Expected CRC: 0x{expected_crc:02X}\n")
    
    # Test different polynomials and configurations
    test_cases = [
        # Current approach (WRONG)
        ("0x1D29 poly (MSB, with ID)", 0x1D, payload, expected_crc, True, False),
        
        # Alternative approaches
        ("0x1D (MSB, no ID)", 0x1D, payload, expected_crc, False, False),
        ("0x1D (MSB, with ID, CRC in calc)", 0x1D, frame_data, expected_crc, True, True),
        
        # Different polynomials
        ("0x85BF poly (the 0x1DA poly!)", 0x85, payload, expected_crc, True, False),
        ("0x07 poly (common CRC-8)", 0x07, payload, expected_crc, True, False),
        ("0x39 poly (CRC-8 DHW)", 0x39, payload, expected_crc, True, False),
        ("0x9B poly (CRC-8 CDMA2000)", 0x9B, payload, expected_crc, True, False),
        
        # No prepended ID
        ("0x1D, no ID, raw payload", 0x1D, payload, expected_crc, False, False),
        
        # Maybe it's LSB?
        ("0x1D (try all 8 bytes)", 0x1D, frame_data[:8], expected_crc, False, True),
    ]
    
    print("Testing different CRC configurations:\n")
    for name, poly, data, exp, has_id, has_crc in test_cases:
        passed, calc, status = test_algorithm(name, poly, data, exp, has_id, has_crc)
        print(f"{status} {name}")
        if data == payload:
            print(f"     Calculated: 0x{calc:02X} (Expected: 0x{exp:02X})")
        else:
            print(f"     Calculated: 0x{calc:02X} (Expected: 0x{exp:02X}) [using full frame]")
        print()
    
    # Try all combinations systematically
    print("\n=== Systematic Polynomial Search ===")
    print("Testing all 256 possible 8-bit polynomials...\n")
    
    found_any = False
    for poly in range(256):
        # Test with ID
        calc_with_id = crc8_msb(bytes([0xD4]) + payload[:7], poly)
        # Test without ID
        calc_without_id = crc8_msb(payload[:7], poly)
        
        if calc_with_id == expected_crc:
            print(f"✓ FOUND: Poly 0x{poly:02X} (with ID) = 0x{calc_with_id:02X}")
            found_any = True
        elif calc_without_id == expected_crc:
            print(f"✓ FOUND: Poly 0x{poly:02X} (without ID) = 0x{calc_without_id:02X}")
            found_any = True
    
    if not found_any:
        print("✗ No standard CRC-8 MSB algorithm matches!")
        print("\nThis suggests:")
        print("  • The CRC algorithm might be LSB-based, not MSB")
        print("  • There might be bit reversal")
        print("  • The algorithm might use a different initialization")
        print("  • Or we're including/excluding the wrong bytes")

if __name__ == "__main__":
    main()
