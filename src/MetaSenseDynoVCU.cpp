#include "MetaSenseDynoVCU.h"

#include <math.h>

namespace {

constexpr float kPiKp = 0.12f;
// Integral gain is in 1/s and multiplied by dt to keep behavior stable across loop rates.
constexpr float kPiKiPerSec = 0.80f;
constexpr float kTorqueClampNm = 250.0f;
constexpr uint32_t kPrechargeDurationMs = 1500;
constexpr float kHvReadyVoltageV = 300.0f;
constexpr float kIdleSetpointMaxRpm = 50.0f;
constexpr float kMotorSetpointMaxRpm = 1500.0f;

} // namespace

namespace MetaSense {

MetaSenseDynoVCU::MetaSenseDynoVCU() = default;

void MetaSenseDynoVCU::begin(uint32_t now_ms)
{
    dynoTorqueDemandNm_ = 0.0f;
    rpmSetpoint_ = 0.0f;
    piIntegrator_ = 0.0f;
    lastUpdateMs_ = now_ms;
    prechargeStartMs_ = now_ms;
    hvArmed_ = false;

    // Phase-1: keep outputs inert; ownership remains in existing HW state machine.
    rb_plus_ = false;
    precharge_ = false;
    ssr_ = false;
    r_minus_ = false;

    LeafCan::reset(leafFb_);
}

float MetaSenseDynoVCU::updatePiLoop(float setpoint, float meas, float dtSec)
{
    if (!isfinite(dtSec) || dtSec < 0.001f) {
        dtSec = 0.001f;
    } else if (dtSec > 0.2f) {
        dtSec = 0.2f;
    }

    const float error = setpoint - meas;
    piIntegrator_ += error * kPiKiPerSec * dtSec;

    float out = error * kPiKp + piIntegrator_;
    if (out > kTorqueClampNm) out = kTorqueClampNm;
    if (out < -kTorqueClampNm) out = -kTorqueClampNm;
    return out;
}

void MetaSenseDynoVCU::update(uint32_t now_ms,
                              float rpm_setpoint,
                              float rpm_meas,
                              float hv_voltage,
                              bool inverter_12v_on)
{
    rpmSetpoint_ = rpm_setpoint;
    const float dtSec = (now_ms >= lastUpdateMs_)
        ? static_cast<float>(now_ms - lastUpdateMs_) / 1000.0f
        : 0.01f;
    dynoTorqueDemandNm_ = updatePiLoop(rpm_setpoint, rpm_meas, dtSec);
    lastUpdateMs_ = now_ms;

    // Match relay intent to selected operating region:
    // Idle  : setpoint <= 50 rpm
    // Motor : 50..1500 rpm (SSR powers inverter bus here)
    // Dyno  : >1500 rpm
    const bool motorSelected =
        (rpm_setpoint > kIdleSetpointMaxRpm) &&
        (rpm_setpoint <= kMotorSetpointMaxRpm);

    // Force all HV outputs off outside Motor-selected path or when inverter 12V is not present.
    if (!inverter_12v_on || !motorSelected) {
        hvArmed_ = false;
        prechargeStartMs_ = now_ms;
        ssr_ = false;
        rb_plus_ = false;
        precharge_ = false;
    } else {
        // SSR must be asserted first in motor precharge sequence.
        ssr_ = true;

        if (!hvArmed_) {
            const bool prechargeDoneByTime = (now_ms - prechargeStartMs_) >= kPrechargeDurationMs;
            const bool prechargeDoneByVoltage = hv_voltage >= kHvReadyVoltageV;
            if (prechargeDoneByTime || prechargeDoneByVoltage) {
                hvArmed_ = true;
            }
        }

        rb_plus_ = hvArmed_;
        precharge_ = !hvArmed_;
    }

    // R- ownership remains in existing state machine for now.
    r_minus_ = false;
}

} // namespace MetaSense
