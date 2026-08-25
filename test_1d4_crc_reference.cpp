#include <cstdio>
#include <cstdint>
#include <cstring>

// CRC-8 MSB with polynomial 0x1D29 (for 0x1D4 frames)
// This is the algorithm we're using in the firmware
uint8_t crc8MsbPoly1D29(const uint8_t* data, uint32_t len) {
    uint8_t crc = 0U;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0U; j < 8U; j++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1U) ^ 0x1DU);  // 0x1D29 >> 8 = 0x1D
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

// Wire CRC algorithm for 0x1D4 (same as 0x1DA but with different poly)
uint8_t computeExact1d4WireCrc(uint8_t idLo, const uint8_t* payload8) {
    uint8_t p[8] = {0U};
    p[0] = idLo;
    memcpy(&p[1], payload8, 7U);
    return crc8MsbPoly1D29(p, 8U);
}

int main(void) {
    printf("=== 0x1D4 CRC Reference Frame Validation ===\n");
    printf("Known-good frames from Thunderstruck TVCU\n\n");
    
    // Reference frames with known-good CRC
    const struct {
        uint8_t data[8];
        uint8_t expectedCrc;
        const char* description;
    } refFrames[] = {
        {{0x6E, 0x6E, 0x00, 0x00, 0x87, 0x44, 0x01, 0x23}, 0x23, "Counter=8 (high), Charge=0x44, Byte5=0x87"},
        {{0x6E, 0x6E, 0x00, 0x00, 0xC7, 0x44, 0x01, 0xE4}, 0xE4, "Counter=C (high), Charge=0x44, Byte5=0xC7"},
        {{0x6E, 0x6E, 0x00, 0x00, 0x07, 0x44, 0x01, 0x28}, 0x28, "Counter=0 (high), Charge=0x44, Byte5=0x07"},
        {{0x6E, 0x6E, 0x00, 0x00, 0x47, 0x44, 0x01, 0xEF}, 0xEF, "Counter=4 (high), Charge=0x44, Byte5=0x47"},
    };
    
    const uint32_t numFrames = sizeof(refFrames) / sizeof(refFrames[0]);
    uint32_t passCount = 0;
    uint32_t failCount = 0;
    
    for (uint32_t i = 0; i < numFrames; i++) {
        const uint8_t* frameData = refFrames[i].data;
        uint8_t expectedCrc = refFrames[i].expectedCrc;
        
        // Calculate CRC using our algorithm
        uint8_t calculatedCrc = computeExact1d4WireCrc(0xD4U, frameData);
        
        // Display frame
        printf("[Frame %u] %s\n", i+1, refFrames[i].description);
        printf("  Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
               frameData[0], frameData[1], frameData[2], frameData[3],
               frameData[4], frameData[5], frameData[6], frameData[7]);
        printf("  Expected CRC:   0x%02X\n", expectedCrc);
        printf("  Calculated CRC: 0x%02X\n", calculatedCrc);
        
        if (calculatedCrc == expectedCrc) {
            printf("  ✓ PASS\n\n");
            passCount++;
        } else {
            printf("  ✗ FAIL - Mismatch!\n\n");
            failCount++;
        }
    }
    
    printf("=== Summary ===\n");
    printf("Passed: %u/%u\n", passCount, numFrames);
    printf("Failed: %u/%u\n", failCount, numFrames);
    
    if (failCount == 0) {
        printf("\n✓ All tests PASSED! CRC algorithm is CORRECT.\n");
        return 0;
    } else {
        printf("\n✗ Tests FAILED! CRC algorithm needs adjustment.\n");
        return 1;
    }
}
