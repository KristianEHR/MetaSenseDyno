// DynoState.h
#pragma once
#include "Telemetry.h"

namespace DynoState {

    enum class Mode {
        Idle,
        Armed,
        Running,
        Fault
    };

    void begin();
    void update(Telemetry &t);   // called each loopOnce()
    void commandStart();
    void commandStop();
    void fault(const char *reason);

    Mode currentMode();
    const char *modeName();
}
