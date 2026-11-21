#pragma once
#include "hal/HalSensors.h"
#include "state/DynoStateMachine.h"
#include "../Telemetry/Telemetry.h"
namespace MetaSense {

class DynoControl {
public:
    bool begin();
    void update(float dtSec);

    void commandTare();
    void commandSetMode(DynoMode mode);

private:
    HalSensors      sensors_;
    DynoStateMachine sm_;
};

} // namespace MetaSense
