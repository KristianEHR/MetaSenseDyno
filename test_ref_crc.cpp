// Test CRC algorithm on known-good Thunderstruck reference frame
// Reference: 6E 6E 00 00 C7 44 01 E4
// Byte 5 (0-indexed): 0x44 = high nibble 4 (counter), low nibble 4 (static)
// Byte 6 (0-indexed): 0x01 = charge status
// Byte 7 (0-indexed): 0xE4 = known-good CRC

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Copy of CRC algorithm
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

int main()
{
    // Known-good reference frame from Thunderstruck VCU
    uint8_t referenceFrame[] = {0x6E, 0x6E, 0x00, 0x00, 0xC7, 0x44, 0x01, 0xE4};
    uint8_t expectedCrc = 0xE4;
    
    // For CRC calculation, set the CRC byte to 0x00
    uint8_t frameForCalc[] = {0x6E, 0x6E, 0x00, 0x00, 0xC7, 0x44, 0x01, 0x00};
    
    // Calculate CRC using our algorithm
    uint8_t calculatedCrc = computeExact1d4LikeCrc(0xD4, frameForCalc);
    
    printf("Reference Frame (1-indexed human notation):\n");
    printf("  Byte 1-2: Motor Amperage (0x6E 0x6E)\n");
    printf("  Byte 3-4: Reserved (0x00 0x00)\n");
    printf("  Byte 5: Torque/Status (0xC7)\n");
    printf("  Byte 6: Static/Relay (0x44)\n");
    printf("  Byte 7: Charge Status (0x01) <-- NOT 0x1E or 0x30!\n");
    printf("  Byte 8: CRC (0x%02X) <-- Expected\n\n", expectedCrc);
    
    printf("CRC Calculation Test:\n");
    printf("  Expected CRC (from reference): 0x%02X\n", expectedCrc);
    printf("  Calculated CRC (our algorithm): 0x%02X\n", calculatedCrc);
    printf("  Match: %s\n\n", (calculatedCrc == expectedCrc) ? "✓ YES" : "✗ NO");
    
    if (calculatedCrc == expectedCrc) {
        printf("✓ Algorithm is CORRECT for 0x1D4 CRC!\n");
        printf("\nKey findings:\n");
        printf("  - Byte 5 (0-indexed [5]): 0x44\n");
        printf("    * High nibble: 4 (rolling counter 0-1-2-3)\n");
        printf("    * Low nibble: 4 (static, NOT 7)\n");
        printf("  - Byte 6 (0-indexed [6]): 0x01 (charge status)\n");
        printf("    * This is DIFFERENT from 0x1E or 0x30!\n");
        return 0;
    } else {
        printf("✗ Algorithm is INCORRECT!\n");
        printf("  Difference: 0x%02X (expected 0x%02X, got 0x%02X)\n", 
               expectedCrc ^ calculatedCrc, expectedCrc, calculatedCrc);
        return 1;
    }
}
