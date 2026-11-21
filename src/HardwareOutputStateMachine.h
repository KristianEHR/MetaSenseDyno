#pragma once

namespace MetaSense::HardwareOutputStateMachine {

// ─────────────────────────────────────────────────────────────────────────
// VCU state machine (rewritten from scratch). This module is the single,
// highest-priority authority over the four HV relay outputs (RB+, RB-, SSR,
// Precharge) and enforces break-before-make (min 20ms) on every relay
// transition. See HardwareOutputStateMachine.cpp for the full state table.
//
// States: READY -> MOTOR / IDLE <-> MOTOR / DYNO, with FAULT reachable from
// IDLE/MOTOR/DYNO when the inverter reports not-ready or live 0x1DA feedback
// is lost. READY runs precharge. Once it succeeds, a setpoint above 250 RPM
// (150 + H, H = 100 RPM) enters MOTOR. An attempt that ends below 300 V is
// retried up to 3 times, then FAULT. After boot, the first time the setpoint
// is 0 RPM opens a latch; READY cannot go operative until that latch is open.
// Transitions (H = 100 RPM):
//   IDLE  -> DYNO   when setpoint > idleThresholdRpm + H (takes priority over MOTOR)
//   IDLE  -> MOTOR  when setpoint > 150 + H
//   MOTOR -> IDLE   when setpoint < 150 - H
//   MOTOR -> DYNO   when engine RPM > 3500 and engine RPM > setpoint + H
//   DYNO  -> IDLE   when setpoint < idleThresholdRpm - H (default 2000; then IDLE -> MOTOR if setpoint > 250)
// Precharge is ON whenever RB- is OFF (READY, MOTOR, and IDLE with RB- off) and OFF whenever RB- is ON (IDLE with RB- on, DYNO). In FAULT every relay (RB+, RB-, SSR,
// precharge) is cleared. FAULT is latched and is left only by a restart
// (FAULT_RESET restarts the controller; boot drives all relay pins LOW). If HV falls to
// 250 V or below in IDLE, DYNO or MOTOR, the machine returns to READY and
// precharge stays on. SSR (motor path) is only closed in
// MOTOR; RB- (grid/dump path) is closed in DYNO, and in IDLE once the setpoint is above
// idleThresholdRpm + H (off below idleThresholdRpm - H, held in between). RB+ (grid
// inverter main power) follows RB- after a 2 s grid precharge dwell.
//
// All HV/fault inputs to this state machine come exclusively from the
// decoded 0x1DA inverter feedback frame (MetaSense::CANBus::feedback()),
// passed in explicitly by the caller -- this module does not read
// CANBus/Input internals directly, and does not accept any bench-forced
// "ready" override. RPM is the gear-ratio-scaled engine RPM (same value
// shown on the dashboard/datalog), not the raw e-motor RPM off the wire.
// The MOTOR PI tracking setpoint is limited by motorModeMaxRpm. The state
// machine clamps the requested setpoint to motorModeMaxRpm + 1000 RPM before
// its transition checks.
// ─────────────────────────────────────────────────────────────────────────

void begin();

void writeThrottle(float percent);
void writeBrake(float percent);
void writeDynoThrottle(float percent);

void update(float engineThrottlePercent,
            float rpmSetpoint,
            float primaryBrakePercent,
            float engineRpm,
            float hvVoltage,
            bool inverterReady,
            bool canFeedbackFresh);

bool isMotorState();
bool isIdleState();
bool isDynoState();
bool isFaultState();

void stop();
const char* stateName();

bool isRbPlusActive();
bool isRbMinusActive();
bool isRbMinusCommandedActive();
bool isMotorPowerActive();
bool isMotorPowerCommandedActive();
bool isInverterPrechargeActive();
bool isInverterPrechargeCommandedActive();
bool isPrechargeSucceeded();

} // namespace MetaSense::HardwareOutputStateMachine
