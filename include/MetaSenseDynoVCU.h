#pragma once

#include <stdint.h>

#include "LeafCan.h"

namespace MetaSense {

class MetaSenseDynoVCU
{
public:
    MetaSenseDynoVCU();

    void begin(uint32_t now_ms);

    // Phase-1 scaffold: no hardware ownership transfer yet.
    void update(uint32_t now_ms,
                float rpm_setpoint,
                float rpm_meas,
                float hv_voltage,
                bool inverter_12v_on);

    // Future HW outputs (kept inert in Phase-1).
    bool getRbPlus() const { return rb_plus_; }
    bool getPrecharge() const { return precharge_; }
    bool getSSR() const { return ssr_; }
    bool getRMinus() const { return r_minus_; }

    const LeafInvFeedback& getLeafFeedback() const { return leafFb_; }

    float getTorqueDemand() const { return dynoTorqueDemandNm_; }

private:
    float updatePiLoop(float setpoint, float meas, float dtSec);

    LeafInvFeedback leafFb_{};

    bool rb_plus_ = false;
    bool precharge_ = false;
    bool ssr_ = false;
    bool r_minus_ = false;

    float dynoTorqueDemandNm_ = 0.0f;
    float rpmSetpoint_ = 0.0f;
    float piIntegrator_ = 0.0f;
    uint32_t lastUpdateMs_ = 0;
    uint32_t prechargeStartMs_ = 0;
    bool hvArmed_ = false;
};

} // namespace MetaSense
