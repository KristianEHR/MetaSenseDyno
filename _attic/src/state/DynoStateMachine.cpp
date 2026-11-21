#include "DynoStateMachine.h"

namespace MetaSense {

void DynoStateMachine::setMode(DynoMode mode) {
    mode_ = mode;
}

void DynoStateMachine::update(const DynoContext& ctx, float /*dtSec*/) {
    if (ctx.fault) {
        mode_ = DynoMode::Fault;
        return;
    }

    switch (mode_) {
    case DynoMode::Idle:
        // wait for command
        break;
    case DynoMode::Warmup:
        // transition criteria
        break;
    case DynoMode::SteadyState:
        // hold torque / rpm
        break;
    case DynoMode::Sweep:
        // ramp setpoint
        break;
    case DynoMode::Cooldown:
        // go back to Idle when cool
        break;
    case DynoMode::Fault:
        // stay until cleared
        break;
    }
}

} // namespace MetaSense
