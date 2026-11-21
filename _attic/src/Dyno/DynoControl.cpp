#include "DynoControl.h"

namespace MetaSense {

bool DynoControl::begin() {
    return sensors_.begin(Wire);
}

void DynoControl::update(float dtSec) {
    EnvSample      env{};
    LoadCellSample load{};
    DynoContext    ctx{};

    sensors_.readEnvironment(env);
    sensors_.readLoadCell(load);

    // Map HAL → DynoContext
    ctx.rpm      = 0.0f;                 // hook RPM HAL later
    ctx.torqueNm = load.kg;              // use kg as torque placeholder
    ctx.tempC    = env.temperatureC;     // correct field name
    ctx.fault    = false;                // no fault logic yet

    sm_.update(ctx, dtSec);
}

void DynoControl::commandTare() {
    sensors_.tareLoadCell();
}

void DynoControl::commandSetMode(DynoMode mode) {
    sm_.setMode(mode);
}

} // namespace MetaSense
