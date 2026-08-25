#!/usr/bin/env python3
"""
Test all 4 reference frames to find the correct polynomial
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

def main():
    print("=== Finding the Correct 0x1D4 CRC Polynomial ===\n")
    
    # All 4 reference frames
    frames = [
        bytes.fromhex("6E 6E 00 00 87 44 01 23"),
        bytes.fromhex("6E 6E 00 00 C7 44 01 E4"),
        bytes.fromhex("6E 6E 00 00 07 44 01 28"),
        bytes.fromhex("6E 6E 00 00 47 44 01 EF"),
    ]
    
    expected_crcs = [0x23, 0xE4, 0x28, 0xEF]
    
    print("Reference frames:")
    for i, (frame, expected) in enumerate(zip(frames, expected_crcs)):
        print(f"  Frame {i+1}: {' '.join(f'{b:02X}' for b in frame[:7])} | CRC: 0x{expected:02X}")
    print()
    
    # Test candidates from previous discovery
    candidates = [
        (0x06, False, "without ID"),
        (0x0D, True, "with ID"),
        (0x3D, False, "without ID"),
        (0x64, False, "without ID"),
        (0x85, False, "without ID"),
        (0x8D, False, "without ID"),
        (0xBC, True, "with ID"),
        (0xD6, False, "without ID"),
        (0xF6, True, "with ID"),
    ]
    
    print("Testing candidate polynomials against all 4 frames:\n")
    
    perfect_matches = []
    
    for poly, with_id, mode in candidates:
        all_pass = True
        results = []
        
        for i, (frame, expected) in enumerate(zip(frames, expected_crcs)):
            if with_id:
                data = bytes([0xD4]) + frame[:7]
            else:
                data = frame[:7]
            
            calc = crc8_msb(data, poly)
            passed = (calc == expected)
            results.append((passed, calc, expected))
            
            if not passed:
                all_pass = False
        
        status = "✓ PERFECT" if all_pass else "✗ FAIL"
        print(f"{status} Poly 0x{poly:02X} ({mode})")
        
        for i, (passed, calc, expected) in enumerate(results):
            symbol = "✓" if passed else "✗"
            print(f"        Frame {i+1}: {symbol} Calc=0x{calc:02X}, Expected=0x{expected:02X}")
        
        if all_pass:
            perfect_matches.append((poly, with_id, mode))
        print()
    
    if perfect_matches:
        print(f"\n✓ FOUND {len(perfect_matches)} perfect match(es):")
        for poly, with_id, mode in perfect_matches:
            id_str = "with 0xD4 frame ID prepended" if with_id else "no frame ID"
            print(f"  • Polynomial 0x{poly:02X} ({mode}, {id_str})")
            
            if len(perfect_matches) == 1:
                print(f"\n✓✓✓ THIS IS THE CORRECT ALGORITHM! ✓✓✓")
                print(f"Use: poly=0x{poly:02X}, include_id={with_id}")
    else:
        print("\n✗ No single polynomial matches all 4 frames!")
        print("This suggests the algorithm might be more complex (LSB, bit reversal, etc)")

if __name__ == "__main__":
    main()
