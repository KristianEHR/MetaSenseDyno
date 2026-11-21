#include "HalRpm.h"

// Replace with your real pin numbers
static const int ENGINE_HALL_PIN = 26;
static const int DRUM_HALL_PIN   = 25;

namespace HalRpm {

void begin() {
    pinMode(ENGINE_HALL_PIN, INPUT_PULLUP);
    pinMode(DRUM_HALL_PIN, INPUT_PULLUP);
}

float readEngineRpm() {
    // TODO: Replace with your real interrupt-based RPM measurement
    return 0.0f;
}

float readDrumRpm() {
    // TODO: Replace with your real interrupt-based RPM measurement
    return 0.0f;
}

}
