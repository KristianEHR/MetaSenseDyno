#pragma once

namespace MetaSense {

enum class DynoMode : uint8_t {
    Idle,
    Warmup,
    SteadyState,
    Sweep,
    Cooldown,
    Fault
};

struct DynoContext {
    float rpm;
    float torqueNm;
    float tempC;
    bool  fault;
};

class DynoStateMachine {
public:
    void setMode(DynoMode mode);
    DynoMode mode() const { return mode_; }

    void update(const DynoContext& ctx, float dtSec);

private:
    DynoMode mode_ = DynoMode::Idle;
};

} // namespace MetaSense
