#pragma once

namespace MetaSense::HardwareOutputStateMachine {

// ─────────────────────────────────────────────────────────────────────────
// VCU state machine (rewritten from scratch). This module is the single,
// highest-priority authority over the four HV relay outputs (RB+, RB-, SSR,
// Precharge) and enforces break-before-make (min 20ms) on every relay
// transition. See HardwareOutputStateMachine.cpp for the full state table.
//
// States: INIT -> START (precharge cycle) -> IDLE <-> MOTOR / DYNO, with
// FAULT reachable from IDLE/MOTOR/DYNO on inverter fault or loss of live
// 0x1DA feedback.
//
// All RPM/HV/fault inputs to this state machine come exclusively from the
// decoded 0x1DA inverter feedback frame (MetaSense::CANBus::feedback()),
// passed in explicitly by the caller -- this module does not read
// CANBus/Input internals directly, and does not accept any bench-forced
// "ready" override.
// ─────────────────────────────────────────────────────────────────────────

void begin();

void writeThrottle(float percent);
void writeBrake(float percent);
void writeDynoThrottle(float percent);

void update(float engineThrottlePercent,
            float rpmSetpoint,
            float primaryBrakePercent,
            float rpmFromCan,
            float hvVoltageFromCan,
            bool inverterFaultFromCan,
            bool canFeedbackFresh);

void requestMotorStartOverride(float rpmSetpoint);

bool isMotorState();
bool isIdleState();
bool isFaultState();

void stop();
const char* stateName();

bool isRbPlusActive();
bool isRbPlusCommandedActive();
void setVcuRelayOverride(bool enabled, bool rbPlus, bool precharge, bool ssr, bool rbMinus);
void applyVcuSimRelayOutputs(bool rbPlusOn, bool prechargeOn, bool ssrOn, bool rbMinusOn);
bool isRbMinusActive();
bool isRbMinusCommandedActive();
bool isSsrActive();
bool isSsrCommandedActive();
bool isPrechargeActive();
bool isPrechargeCommandedActive();
bool isPrechargeSucceeded();

} // namespace MetaSense::HardwareOutputStateMachine
