#include "DynoState.h"
#include "../Telemetry/Telemetry.h"

namespace DynoState {

static Mode current = Mode::Idle;
static const char *faultReason = nullptr;

void begin() {
    current = Mode::Idle;
    faultReason = nullptr;
}

void update(const MetaSense::Telemetry &t) {
    // If Leaf inverter reports a fault, enter Fault mode
    if (t.leaf_invFault) {
        current = Mode::Fault;
        faultReason = "Leaf inverter fault";
        return;
    }

    // Otherwise nothing to do for now
}

void commandStart() {
    if (current != Mode::Fault) {
        current = Mode::Running;
    }
}

void commandStop() {
    if (current != Mode::Fault) {
        current = Mode::Idle;
    }
}

void fault(const char *reason) {
    current = Mode::Fault;
    faultReason = reason;
}

Mode currentMode() {
    return current;
}

const char *modeName() {
    switch (current) {
        case Mode::Idle:    return "Idle";
        case Mode::Running: return "Running";
        case Mode::Fault:   return faultReason ? faultReason : "Fault";
        default:            return "Unknown";
    }
}

} // namespace DynoState
