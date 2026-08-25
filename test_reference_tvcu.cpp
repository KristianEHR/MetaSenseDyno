#include <cstdint>
#include <cstring>
#include <cstdio>

// Test CRC algorithm against known-good Thunderstruck TVCU frames

uint8_t crc8MsbPoly1D29(const uint8_t* data, size_t len) {
    uint8_t crc = 0U;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80U) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x1DU);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }
    return crc;
}

uint8_t computeExact1d4LikeCrc(uint8_t idLo, const uint8_t* payload8) {
    uint8_t p[8] = {0U};
    p[0] = idLo;
    memcpy(&p[1], payload8, 7U);
    return crc8MsbPoly1D29(p, 8U);
}

int main() {
    // Known-good reference frames from Thunderstruck TVCU
    struct Frame {
        uint8_t idx;
        uint8_t data[8];
        uint8_t expected_crc;
    };

    Frame frames[] = {
        { 6, {0x6E, 0x6E, 0x00, 0x00, 0x87, 0x44, 0x01, 0x00}, 0x23},
        { 7, {0x6E, 0x6E, 0x00, 0x00, 0xC7, 0x44, 0x01, 0x00}, 0xE4},
        { 8, {0x6E, 0x6E, 0x00, 0x00, 0x07, 0x44, 0x01, 0x00}, 0x28},
        { 9, {0x6E, 0x6E, 0x00, 0x00, 0x47, 0x44, 0x01, 0x00}, 0xEF},
    };

    printf("Testing CRC algorithm against known-good Thunderstruck TVCU frames:\n");
    printf("============================================================\n\n");

    int pass = 0;
    int fail = 0;

    for (const auto& frame : frames) {
        uint8_t calc_crc = computeExact1d4LikeCrc(0xD4U, frame.data);
        bool match = (calc_crc == frame.expected_crc);

        printf("Frame %2d: ", frame.idx);
        printf("Data=%02X %02X %02X %02X %02X %02X %02X | ", 
               frame.data[0], frame.data[1], frame.data[2], frame.data[3],
               frame.data[4], frame.data[5], frame.data[6]);
        printf("Expected CRC=0x%02X | Calculated=0x%02X | %s\n",
               frame.expected_crc, calc_crc, match ? "✓ PASS" : "✗ FAIL");

        if (match) {
            pass++;
        } else {
            fail++;
        }
    }

    printf("\n============================================================\n");
    printf("Results: %d PASS, %d FAIL\n", pass, fail);

    if (fail == 0) {
        printf("\n✓ CRC algorithm is CORRECT! All test frames validated.\n");
        printf("\nKey observations from reference frames:\n");
        printf("  - Byte 5 low nibble = 0x7 (static, not 0x4)\n");
        printf("  - Byte 6 (charge) = 0x44 (static)\n");
        printf("  - Byte 5 high nibble cycles: 8→C→0→4 (rolling counter)\n");
        printf("  - Byte 7 = CRC (0x23, 0xE4, 0x28, 0xEF pattern)\n");
    } else {
        printf("\n✗ CRC algorithm MISMATCH! Need to review polynomial/implementation.\n");
    }

    return fail == 0 ? 0 : 1;
}
