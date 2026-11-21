#pragma once
#include "../Telemetry/Telemetry.h"

namespace DynoState {

    enum class Mode {
        Idle,
        Running,
        Fault
    };

    // Initialize state machine
    void begin();

    // Update state machine based on telemetry
    void update(const MetaSense::Telemetry &t);

    // Commands
    void commandStart();
    void commandStop();
    void fault(const char *reason);

    // Query
    Mode currentMode();
    const char *modeName();

} // namespace DynoState
