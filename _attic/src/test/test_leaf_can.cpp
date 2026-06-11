#include <Arduino.h>
#include <unity.h>
#include "LeafCan.h"
#include "Telemetry.h"

void test_decode_inv_voltage() {
    Telemetry t{};
    CanFrame frame{};

    // Example: ID and payload you know from your ZE1 inverter
    frame.id  = 0x1DA;   // replace with real ID
    frame.dlc = 8;
    frame.data[0] = 0x12;
    frame.data[1] = 0x34;
    frame.data[2] = 0x56;
    frame.data[3] = 0x78;
    frame.data[4] = 0x9A;
    frame.data[5] = 0xBC;
    frame.data[6] = 0xDE;
    frame.data[7] = 0xF0;

    bool ok = LeafCan::decode(frame, t);
    TEST_ASSERT_TRUE(ok);

    // Replace with expected engineering values
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 360.0f, t.dcBusVoltage);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_decode_inv_voltage);
    UNITY_END();
}

void loop() {
    // not used
}
