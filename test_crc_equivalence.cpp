#include <cstdint>
#include <cstdio>
#include <cstring>

// 0x1DA algorithm: poly 0x85, init 0x00, xorOut 0xBF
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

// 0x1D4 algorithm: poly 0x1D, init 0x00, xorOut 0x29
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

int main() {
    printf("Testing if crc8MsbPoly85BF and crc8MsbPoly1D29 are equivalent:\n\n");

    // Test with several data patterns
    uint8_t test_data[8] = {0xDA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    for (int pattern = 0; pattern < 256; pattern += 16) {
        memset(test_data, pattern, 8);
        test_data[0] = 0xDA;  // Keep first byte as frame ID
        
        uint8_t crc_85 = crc8MsbPoly85BF(test_data, 7);  // First 7 bytes (not including CRC position)
        uint8_t crc_1d = crc8MsbPoly1D29(test_data, 7);
        
        printf("Pattern 0x%02X: Poly85=%02X, Poly1D=%02X %s\n", 
               pattern, crc_85, crc_1d,
               (crc_85 == crc_1d) ? "✓ SAME" : "✗ DIFFERENT");
    }
    
    printf("\n");
    printf("Test specific data patterns:\n");
    
    // Real-world 0x1DA frame example
    uint8_t frame_1da[] = {0xDA, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
    uint8_t crc_85_1 = crc8MsbPoly85BF(frame_1da, 7);
    uint8_t crc_1d_1 = crc8MsbPoly1D29(frame_1da, 7);
    printf("Frame [DA 10 20 30 40 50 60]:\n");
    printf("  Poly85 CRC: 0x%02X\n", crc_85_1);
    printf("  Poly1D CRC: 0x%02X\n", crc_1d_1);
    printf("  Result: %s\n\n", (crc_85_1 == crc_1d_1) ? "SAME" : "DIFFERENT");
    
    // Inverse check: what if we include the CRC byte?
    uint8_t frame_with_crc[8];
    memcpy(frame_with_crc, frame_1da, 7);
    frame_with_crc[7] = crc_85_1;
    
    printf("Full frame [DA 10 20 30 40 50 60 %02X]:\n", crc_85_1);
    uint8_t crc_85_check = crc8MsbPoly85BF(frame_with_crc, 8);
    uint8_t crc_1d_check = crc8MsbPoly1D29(frame_with_crc, 8);
    printf("  Poly85 CRC check: 0x%02X (expect 0 if valid)\n", crc_85_check);
    printf("  Poly1D CRC check: 0x%02X\n", crc_1d_check);
    
    return 0;
}
