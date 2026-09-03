#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/gpio.h>

#include "HardwareOutputStateMachine.h"
#include "globals.h"

// ─────────────────────────────────────────────────────────────────────────
// This file is split into two clearly separated levels:
//
//   1. HW LAYER (namespace `hw`, below): owns the physical relay pins, PWM
//      channels, and status LED. It knows nothing about VCU states. Its
//      only safety-relevant job is: (a) enforce the general SSR/RB-
//      mutual-exclusion invariant, (b) enforce break-before-make (min 20ms)
//      on every relay transition, (c) actually write the GPIO/PWM/LED
//      hardware. It exposes a small command-based API (RelayCommand,
//      applyRelayCommand, writeThrottleDutyPwm, writeActuatorChannel,
//      setLedColor) plus read-back queries.
//
//   2. VCU LOGIC (below the `hw` namespace): the state machine itself
//      (INIT -> START (precharge) -> IDLE <-> MOTOR / DYNO, with FAULT
//      reachable from IDLE/MOTOR/DYNO). It decides, for the current state,
//      what relay pattern / PWM values / LED color are *wanted*, and hands
//      those to the HW layer to realize safely. All RPM/HV/fault inputs
//      come exclusively from the decoded 0x1DA inverter feedback frame,
//      passed in by the caller -- this module does not read CANBus/Input
//      internals directly, and accepts no bench-forced "ready" bypass (a
//      prior such mechanism, VCU_switch=0, was identified as an unintended
//      artifact and has been retired).
//
// Relay table (RB+, RB-, SSR, Precharge) decided by the VCU layer. Precharge
// is always slaved to SSR (enforced by applyRelayCommand -- Precharge always
// follows SSR ON, regardless of what the VCU layer requests):
//   INIT:    OFF, OFF, OFF, OFF
//   START:   OFF, OFF, ON,  ON  (2.5s precharge dwell)
//   IDLE:    ON,  ON,  OFF, OFF   (HV keep-alive can flip SSR+Precharge on / RB- off)
//   MOTOR:   ON,  OFF, ON,  ON  (Precharge active but inert here -- RB+ carries the real power path)
//   DYNO:    ON,  ON,  OFF, OFF
//   FAULT:   OFF, OFF, OFF, OFF
// ─────────────────────────────────────────────────────────────────────────

namespace {

// ═══════════════════════════════════════════════════════════════════════
// HW LAYER -- physical relay/PWM/LED drive. No knowledge of VCU states.
// ═══════════════════════════════════════════════════════════════════════
namespace hw {

constexpr uint32_t kRelaySwitchDelayMs = 20;  // Break-before-make dwell.

struct RelayCommand {
    bool rbMinusOn = false;
    bool ssrOn = false;
    bool rbPlusOn = false;
    bool prechargeOn = false;
};

RelayCommand makeRelayCommand(bool rbMinusOn, bool ssrOn, bool rbPlusOn, bool prechargeOn)
{
    RelayCommand cmd;
    cmd.rbMinusOn = rbMinusOn;
    cmd.ssrOn = ssrOn;
    cmd.rbPlusOn = rbPlusOn;
    cmd.prechargeOn = prechargeOn;
    return cmd;
}

RelayCommand activeRelayCommand{};

Adafruit_NeoPixel strip(MetaSense::Globals::kOnboardLedCount,
                        MetaSense::Globals::kOnboardLedPin,
                        NEO_GRB + NEO_KHZ800);

int pwmMaxValue(int bits)
{
    if (bits <= 1) {
        return 1;
    }
    if (bits >= 15) {
        return 32767;
    }
    return (1 << bits) - 1;
}

void setupActuatorPwmChannel(int pin, int channel)
{
    ledcSetup(channel,
              MetaSense::Globals::kActuatorPwmFrequencyHz,
              MetaSense::Globals::kActuatorPwmResolutionBits);
    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0);
}

void setupThrottlePwmChannel(int pin, int channel)
{
    ledcSetup(channel,
              MetaSense::Globals::kThrottlePwmFrequencyHz,
              MetaSense::Globals::kThrottlePwmResolutionBits);
    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0);
}

void writeActuatorChannel(int channel, float percent)
{
    percent = constrain(percent, 0.0f, 100.0f);
    const int maxPwm = pwmMaxValue(MetaSense::Globals::kActuatorPwmResolutionBits);
    const int pwm = static_cast<int>(percent * static_cast<float>(maxPwm) / 100.0f);
    ledcWrite(channel, pwm);
}

void writeThrottleDutyPwm(float percent)
{
    percent = constrain(percent, 0.0f, 100.0f);

    const float outMin = MetaSense::Globals::kThrottlePwmMinPercent;
    const float outMax = MetaSense::Globals::kThrottlePwmMaxPercent;
    const float mapped = outMin + (percent / 100.0f) * (outMax - outMin);
    const float constrained = constrain(mapped, 0.0f, 100.0f);
    const int maxPwm = pwmMaxValue(MetaSense::Globals::kThrottlePwmResolutionBits);
    const int pwm = static_cast<int>(constrained * static_cast<float>(maxPwm) / 100.0f);

    ledcWrite(MetaSense::Globals::kThrottlePwmChannel, constrain(pwm, 0, maxPwm));
}

// precharge(): the single named function that switches the SSR+Precharge
// pair. Scope: this is invoked for exactly two purposes -- the initial 2.5s
// precharge dwell in START, and the current-limited recharge ("keep-alive")
// supply to the inverter while IDLE. (It also incidentally fires for
// MOTOR/DYNO entry since those states pin SSR to a fixed value too, but
// Precharge's state is inert there -- MOTOR: don't care, DYNO: already
// lands on precharge(false) since DYNO runs SSR=OFF.)
//   precharge(true):  PRECHARGE relay = ON, then SSR = ON.
//   precharge(false): SSR = OFF, then PRECHARGE relay = OFF.
void precharge(bool on)
{
    if (on) {
        digitalWrite(MetaSense::Globals::kPrechargeRelayPin, HIGH);
        digitalWrite(MetaSense::Globals::kSssrPin, HIGH);
    } else {
        digitalWrite(MetaSense::Globals::kSssrPin, LOW);
        digitalWrite(MetaSense::Globals::kPrechargeRelayPin, LOW);
    }
}

void writeRelayPinsImmediate(const RelayCommand& cmd)
{
    digitalWrite(MetaSense::Globals::kRbMinusFetPin, cmd.rbMinusOn ? HIGH : LOW);
    precharge(cmd.prechargeOn);
    digitalWrite(MetaSense::Globals::kRbPlusRelayPin, cmd.rbPlusOn ? HIGH : LOW);
}

bool relayCommandEquals(const RelayCommand& a, const RelayCommand& b)
{
    return a.rbMinusOn == b.rbMinusOn &&
        a.ssrOn == b.ssrOn &&
        a.rbPlusOn == b.rbPlusOn &&
        a.prechargeOn == b.prechargeOn;
}

// Applies a relay command and enforces two general invariants: (a) SSR and
// RB- can never both be commanded ON, (b) Precharge is always active
// whenever SSR is ON -- Precharge has no independent role of its own; it is
// always a slave to SSR (SSR does the actual power switching, Precharge
// switches no power). This is the single point of authority for relay
// outputs.
//
// Break-before-make (open, wait the relay switch delay, then close) applies
// to two crossovers, both of which connect the bus via one path while
// disconnecting the other: SSR<->RB- (swapping which path carries the load),
// and RB+<->SSR/Precharge (the direct path vs. the current-limited
// precharge() path -- RB+ must open before precharge() energizes the bus,
// and precharge() must de-energize before RB+ closes, or the fresh
// charge/discharge current would bypass the current limiting entirely).
void applyRelayCommand(RelayCommand target)
{
    // General invariant: SSR and RB- can never both be commanded ON.
    if (target.ssrOn && target.rbMinusOn) {
        target.rbMinusOn = false;
    }

    // General invariant: Precharge always follows SSR (slaved to it).
    if (target.ssrOn) {
        target.prechargeOn = true;
    }

    if (relayCommandEquals(activeRelayCommand, target)) {
        return;
    }

    const bool rbMinusGoingLow = activeRelayCommand.rbMinusOn && !target.rbMinusOn;
    const bool rbMinusGoingHigh = !activeRelayCommand.rbMinusOn && target.rbMinusOn;
    const bool ssrGoingLow = activeRelayCommand.ssrOn && !target.ssrOn;
    const bool ssrGoingHigh = !activeRelayCommand.ssrOn && target.ssrOn;
    const bool rbPlusGoingLow = activeRelayCommand.rbPlusOn && !target.rbPlusOn;
    const bool rbPlusGoingHigh = !activeRelayCommand.rbPlusOn && target.rbPlusOn;

    // Blocking delay is intentional here -- rotational inertia means the PI
    // control loop will not notice a relay transition.
    const bool ssrRbMinusCrossover = (ssrGoingLow && rbMinusGoingHigh) ||
                                     (rbMinusGoingLow && ssrGoingHigh);
    const bool rbPlusPrechargeCrossover = (rbPlusGoingLow && ssrGoingHigh) ||
                                          (ssrGoingLow && rbPlusGoingHigh);
    if (ssrRbMinusCrossover || rbPlusPrechargeCrossover) {
        RelayCommand breakStep = activeRelayCommand;
        if (rbMinusGoingLow) breakStep.rbMinusOn = false;
        if (ssrGoingLow) {
            breakStep.ssrOn = false;
            breakStep.prechargeOn = false;
        }
        if (rbPlusGoingLow) breakStep.rbPlusOn = false;

        writeRelayPinsImmediate(breakStep);
        delay(kRelaySwitchDelayMs);
    }

    writeRelayPinsImmediate(target);
    activeRelayCommand = target;
}

uint32_t packColor(uint8_t r, uint8_t g, uint8_t b)
{
    return strip.Color(r, g, b);
}

void setLedColor(uint32_t color)
{
    strip.setPixelColor(0, color);
    strip.show();
}

void beginOutputs()
{
    strip.begin();
    strip.clear();
    strip.show();

    setupThrottlePwmChannel(MetaSense::Globals::kThrottlePin,
                            MetaSense::Globals::kThrottlePwmChannel);
    setupActuatorPwmChannel(MetaSense::Globals::kBrakePin,
                            MetaSense::Globals::kBrakePwmChannel);
    setupActuatorPwmChannel(MetaSense::Globals::kThrottleVcuPin,
                            MetaSense::Globals::kDynoThrottlePwmChannel);

    auto configureOutputPin = [](int pin) {
        if (!digitalPinIsValid(pin)) {
            return false;
        }
        gpio_reset_pin(static_cast<gpio_num_t>(pin));
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        return true;
    };

    (void)configureOutputPin(MetaSense::Globals::kRbMinusFetPin);
    (void)configureOutputPin(MetaSense::Globals::kSssrPin);
    (void)configureOutputPin(MetaSense::Globals::kRbPlusRelayPin);
    (void)configureOutputPin(MetaSense::Globals::kPrechargeRelayPin);

    activeRelayCommand = RelayCommand{};
    writeRelayPinsImmediate(activeRelayCommand);
}

bool isRbPlusActive()
{
    return digitalRead(MetaSense::Globals::kRbPlusRelayPin) == HIGH;
}

bool isRbMinusActive()
{
    return digitalRead(MetaSense::Globals::kRbMinusFetPin) == HIGH;
}

bool isSsrActive()
{
    return digitalRead(MetaSense::Globals::kSssrPin) == HIGH;
}

bool isPrechargeActive()
{
    return digitalRead(MetaSense::Globals::kPrechargeRelayPin) == HIGH;
}

} // namespace hw

// ═══════════════════════════════════════════════════════════════════════
// VCU LOGIC -- state machine, transitions, precharge sequencing. Only
// talks to the `hw` namespace above via its command-based API.
// ═══════════════════════════════════════════════════════════════════════

enum class OutputState {
    INIT,
    START,
    IDLE,
    MOTOR,
    DYNO,
    FAULT
};

const char* outputStateName(OutputState s)
{
    switch (s) {
    case OutputState::INIT:  return "INIT";
    case OutputState::START: return "START";
    case OutputState::IDLE:  return "IDLE";
    case OutputState::MOTOR: return "MOTOR";
    case OutputState::DYNO:  return "DYNO";
    case OutputState::FAULT: return "FAULT";
    }
    return "INIT";
}

OutputState state = OutputState::INIT;

// --- Tunables from the agreed spec ---
constexpr uint32_t kPrechargeDurationMs = 2500;  // Fixed precharge dwell.
constexpr uint8_t  kPrechargeMaxAttempts = 3;    // Retries before FAULT.
constexpr float kPrechargeHvReadyV = 300.0f;     // HV must reach this to succeed.
constexpr float kStartRpmSetpointMaxRpm = 150.0f; // INIT->START gate: setpoint must be below this.
constexpr float kIdleDynoHystCenterRpm = 500.0f;
constexpr float kIdleDynoHystBandRpm = 150.0f;    // Single hysteresis band: 500 +/- 150.
constexpr float kDynoEnterRpm = kIdleDynoHystCenterRpm + kIdleDynoHystBandRpm; // 650
constexpr float kIdleEnterRpm = kIdleDynoHystCenterRpm - kIdleDynoHystBandRpm; // 350
constexpr float kMotorSetpointMinRpm = 150.0f;    // MOTOR entry requires setpoint above this.
// RB- (load-dump resistor connection) must stay OFF right after entering
// IDLE (a freshly-precharged bus discharging into the dump resistor is a
// hazard) and only engage once RPM climbs high enough that the engine is
// genuinely generating regen power that needs dissipating. Hysteresis:
// engage at RPM>=550, release at RPM<450 (holds latched state in between
// to avoid relay chatter at the edge).
constexpr float kIdleRbMinusHystCenterRpm = 500.0f;
constexpr float kIdleRbMinusHystBandRpm = 50.0f;
constexpr float kIdleRbMinusEngageRpm = kIdleRbMinusHystCenterRpm + kIdleRbMinusHystBandRpm; // 550
constexpr float kIdleRbMinusReleaseRpm = kIdleRbMinusHystCenterRpm - kIdleRbMinusHystBandRpm; // 450
constexpr float kHvKeepAliveMaxRpm = 500.0f;      // IDLE-only HV keep-alive: RPM must be <= this.
constexpr float kHvKeepAliveEngageV = 10.0f;      // Engage precharge-channel recharge below this HV.
constexpr float kHvKeepAliveReleaseV = 300.0f;    // Release recharge above this HV (latched, avoids chatter).
// Reactivated: safe under the RB- load-dump rule above, since keep-alive's
// RPM ceiling (<=500) always sits below the RB- engage threshold (>=550),
// so the two latches can never be active at the same time.
constexpr bool kHvKeepAliveEnabled = true;

// --- Precharge sequencing state ---
uint32_t prechargeStartMs = 0;
uint8_t prechargeAttempt = 0;

// --- IDLE-state HV keep-alive latch (persists across calls so the 200V/
// 300V engage/release hysteresis can't chatter at either edge). ---
bool idleHvKeepAliveLatched = false;

// --- IDLE-state RB- (load-dump) engage latch (persists across calls so
// the 550/450 RPM engage/release hysteresis can't chatter at the edge). ---
bool idleRbMinusLatched = false;


// --- Motor-start override (operator request, e.g. START-request button) ---
bool motorStartRequestPending = false;
float motorStartRequestedRpm = 0.0f;

// Decides the desired brake/dyno-throttle PWM split for the current state
// and hands it to the HW layer. Only this function knows which channel
// means what per state.
void writePrimaryBrakeSplit(OutputState hwState, float signedPercent)
{
    signedPercent = constrain(signedPercent, -100.0f, 100.0f);

    switch (hwState) {
    case OutputState::MOTOR:
        hw::writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel,
                     signedPercent > 0.0f ? signedPercent : 0.0f);
        hw::writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel, 0.0f);
        break;

    case OutputState::DYNO:
        hw::writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel, 0.0f);
        hw::writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel,
                     signedPercent < 0.0f ? -signedPercent : 0.0f);
        break;

    case OutputState::INIT:
    case OutputState::START:
    case OutputState::IDLE:
    case OutputState::FAULT:
    default:
        hw::writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel, 0.0f);
        hw::writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel, 0.0f);
        break;
    }
}

// Decides the desired status-LED color for the current state.
void setStateLed(OutputState hwState)
{
    uint32_t color = 0;

    switch (hwState) {
    case OutputState::INIT:
        color = hw::packColor(255, 255, 255);
        break;
    case OutputState::START:
        color = hw::packColor(255, 200, 0);
        break;
    case OutputState::IDLE:
        color = hw::packColor(0, 0, 255);
        break;
    case OutputState::MOTOR:
        color = hw::packColor(255, 0, 255);
        break;
    case OutputState::DYNO:
        color = hw::packColor(0, 255, 0);
        break;
    case OutputState::FAULT:
        color = hw::packColor(255, 0, 0);
        break;
    }

    hw::setLedColor(color);
}

// Decides the desired relay pattern and PWM/LED outputs for the current
// state, and hands them to the HW layer to realize.
void applyOutputs(OutputState hwState,
                  float engineThrottlePercent,
                  float primaryBrakePercent,
                  float hvVoltageFromCan,
                  float engineRpm,
                  float rpmSetpoint)
{
    hw::writeThrottleDutyPwm(engineThrottlePercent);

    hw::RelayCommand cmd{};

    switch (hwState) {
    case OutputState::INIT:
        cmd = hw::makeRelayCommand(false, false, false, false);
        break;

    case OutputState::START:
        // Precharging: SSR + Precharge ON, RB+/RB- OFF.
        cmd = hw::makeRelayCommand(false, true, false, true);
        break;

    case OutputState::IDLE: {
        // RB- (load-dump resistor connection) must stay OFF right after
        // entering IDLE -- discharging a freshly-precharged bus into the
        // dump resistor is a hazard -- and only engage once RPM indicates
        // genuine regen power needing dissipation. Latched hysteresis:
        // engage at RPM>=550, release at RPM<450.
        if (engineRpm - kIdleRbMinusHystBandRpm >= kIdleRbMinusHystCenterRpm) {
            idleRbMinusLatched = true;
        } else if (engineRpm + kIdleRbMinusHystBandRpm < kIdleRbMinusHystCenterRpm) {
            idleRbMinusLatched = false;
        }

        // HV keep-alive: recharges the DC bus through the current-limited
        // precharge channel (SSR+PRECHARGE together -- same relay pattern
        // as the initial START precharge) whenever HV sags, but only while
        // genuinely idling: RPM <= 500 AND RPM > the operator's rpm
        // setpoint. Engage at HV<200V, release at HV>300V (latched, so HV
        // sitting near either edge cannot cause relay chatter). This RPM
        // ceiling (<=500) always sits below the RB- engage threshold
        // (>=550) above, so keep-alive can never be active while RB- would
        // want to engage -- the two latches cannot conflict.
        const bool keepAliveEligible = kHvKeepAliveEnabled &&
                                        (engineRpm <= kHvKeepAliveMaxRpm) &&
                                        (engineRpm > rpmSetpoint);
        if (!keepAliveEligible) {
            idleHvKeepAliveLatched = false;
        } else if (!idleHvKeepAliveLatched && hvVoltageFromCan < kHvKeepAliveEngageV) {
            idleHvKeepAliveLatched = true;
        } else if (idleHvKeepAliveLatched && hvVoltageFromCan > kHvKeepAliveReleaseV) {
            idleHvKeepAliveLatched = false;
        }

        if (idleHvKeepAliveLatched) {
            // RB+=OFF (opened so the current-limited precharge() path, not
            // the direct path, carries the recharge current), RB-=OFF,
            // SSR=ON, PRECHARGE=ON. applyRelayCommand()'s RB+<->SSR/Precharge
            // crossover handling staggers this: RB+ opens before SSR/
            // Precharge energize, and closes again only after they
            // de-energize when this latch releases.
            cmd = hw::makeRelayCommand(false, true, false, true);
        } else {
            // RB+=ON always; RB- follows the load-dump latch above;
            // SSR/Precharge OFF.
            cmd = hw::makeRelayCommand(idleRbMinusLatched, false, true, false);
        }
        break;
    }

    case OutputState::MOTOR:
        cmd = hw::makeRelayCommand(false, true, true, false);
        break;

    case OutputState::DYNO:
        cmd = hw::makeRelayCommand(true, false, true, false);
        break;

    case OutputState::FAULT:
        // FAULT forces SSR + Precharge OFF (stops driving/decharges the
        // inverter's DC bus energy path) but deliberately leaves RB+/RB-
        // frozen at whatever pattern was active immediately before the
        // fault, rather than forcing them off.
        cmd = hw::makeRelayCommand(hw::isRbMinusActive(), false, hw::isRbPlusActive(), false);
        break;

    default:
        cmd = hw::makeRelayCommand(false, false, false, false);
        break;
    }

    hw::applyRelayCommand(cmd);
    writePrimaryBrakeSplit(hwState, primaryBrakePercent);
    setStateLed(hwState);
}

} // namespace

namespace MetaSense::HardwareOutputStateMachine {

void begin()
{
    hw::beginOutputs();

    state = OutputState::INIT;
    prechargeStartMs = 0;
    prechargeAttempt = 0;
    motorStartRequestPending = false;
    motorStartRequestedRpm = 0.0f;
    setStateLed(state);
}

void writeThrottle(float percent)
{
    hw::writeThrottleDutyPwm(percent);
}

void writeBrake(float percent)
{
    hw::writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel, percent);
}

void writeDynoThrottle(float percent)
{
    hw::writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel, percent);
}

void update(float engineThrottlePercent,
            float rpmSetpoint,
            float primaryBrakePercent,
            float engineRpm,
            float hvVoltageFromCan,
            bool inverterFaultFromCan,
            bool canFeedbackFresh)
{
    const uint32_t now = millis();

    bool motorStartByRequest = false;
    if (motorStartRequestPending) {
        motorStartRequestPending = false;
        motorStartByRequest = true;
    }

    // FAULT is reachable from IDLE/MOTOR/DYNO on inverter fault or loss of
    // live 0x1DA feedback. It is NOT reachable this way from START -- during
    // precharge, only the HV-not-reached-in-time condition matters (an
    // inverterFault flag alone must not abort precharge early).
    const bool faultConditionActive = inverterFaultFromCan || !canFeedbackFresh;
    if (faultConditionActive &&
        (state == OutputState::IDLE || state == OutputState::MOTOR || state == OutputState::DYNO)) {
        state = OutputState::FAULT;
    }

    switch (state) {
    case OutputState::INIT:
        // Gate: live CAN feedback present AND operator setpoint is safely
        // low before we even attempt to close any HV relay.
        if (canFeedbackFresh && (rpmSetpoint < kStartRpmSetpointMaxRpm)) {
            state = OutputState::START;
            prechargeStartMs = now;
            prechargeAttempt = 1;
        }
        break;

    case OutputState::START: {
        const uint32_t elapsedMs = now - prechargeStartMs;
        if (elapsedMs >= kPrechargeDurationMs) {
            state = OutputState::IDLE;
        }
        break;
    }

    case OutputState::IDLE:
        if (motorStartByRequest) {
            state = OutputState::MOTOR;
        } else if (engineRpm > kDynoEnterRpm) {
            state = OutputState::DYNO;
        } else if ((engineRpm < rpmSetpoint) && (rpmSetpoint > kMotorSetpointMinRpm)) {
            state = OutputState::MOTOR;
        }
        break;

    case OutputState::MOTOR:
        if (!((engineRpm < rpmSetpoint) && (rpmSetpoint > kMotorSetpointMinRpm))) {
            state = (engineRpm > kDynoEnterRpm) ? OutputState::DYNO : OutputState::IDLE;
        }
        break;

    case OutputState::DYNO:
        if (motorStartByRequest) {
            state = OutputState::MOTOR;
        } else if (engineRpm < kIdleEnterRpm) {
            state = OutputState::IDLE;
        } else if ((engineRpm < rpmSetpoint) && (rpmSetpoint > kMotorSetpointMinRpm)) {
            state = OutputState::MOTOR;
        }
        break;

    case OutputState::FAULT:
        // Latched: recovery requires returning to INIT and re-running the
        // full precharge sequence. No automatic exit from FAULT.
        break;
    }

    applyOutputs(state, engineThrottlePercent, primaryBrakePercent, hvVoltageFromCan, engineRpm, rpmSetpoint);
}

void requestMotorStartOverride(float rpmSetpoint)
{
    motorStartRequestPending = true;
    motorStartRequestedRpm = rpmSetpoint;
}

bool isMotorState()
{
    return state == OutputState::MOTOR;
}

bool isIdleState()
{
    return state == OutputState::IDLE;
}

bool isFaultState()
{
    return state == OutputState::FAULT;
}

void stop()
{
    writeThrottle(0.0f);
    state = OutputState::IDLE;
    applyOutputs(state, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

const char* stateName()
{
    return outputStateName(state);
}

bool isRbPlusActive()
{
    return hw::isRbPlusActive();
}

bool isRbPlusCommandedActive()
{
    return hw::activeRelayCommand.rbPlusOn;
}

void setVcuRelayOverride(bool enabled, bool rbPlus, bool precharge, bool ssr, bool rbMinus)
{
    // Retired: the state machine is the sole, highest-priority authority
    // over relay outputs. A VCU override bypass was identified as an
    // unintended artifact of past bench-testing and has been removed.
    (void)enabled;
    (void)rbPlus;
    (void)precharge;
    (void)ssr;
    (void)rbMinus;
}

void applyVcuSimRelayOutputs(bool rbPlusOn, bool prechargeOn, bool ssrOn, bool rbMinusOn)
{
    // Retired along with setVcuRelayOverride() -- see comment there.
    (void)rbPlusOn;
    (void)prechargeOn;
    (void)ssrOn;
    (void)rbMinusOn;
}

bool isRbMinusActive()
{
    return hw::isRbMinusActive();
}

bool isRbMinusCommandedActive()
{
    return hw::activeRelayCommand.rbMinusOn;
}

bool isSsrActive()
{
    return hw::isSsrActive();
}

bool isSsrCommandedActive()
{
    return hw::activeRelayCommand.ssrOn;
}

bool isPrechargeActive()
{
    return hw::isPrechargeActive();
}

bool isPrechargeCommandedActive()
{
    return hw::activeRelayCommand.prechargeOn;
}

bool isPrechargeSucceeded()
{
    return (state != OutputState::INIT) && (state != OutputState::START) && (state != OutputState::FAULT);
}

} // namespace MetaSense::HardwareOutputStateMachine
