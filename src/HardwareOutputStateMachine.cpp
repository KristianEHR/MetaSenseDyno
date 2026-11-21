#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/gpio.h>

#include "HardwareOutputStateMachine.h"
#include "Settings.h"
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
//      (READY (precharge) -> MOTOR / IDLE / DYNO, with FAULT
//      reachable from IDLE/MOTOR/DYNO). It decides, for the current state,
//      what relay pattern / PWM values / LED color are *wanted*, and hands
//      those to the HW layer to realize safely. All RPM/HV/fault inputs
//      come exclusively from the decoded 0x1DA inverter feedback frame,
//      passed in by the caller -- this module does not read CANBus/Input
//      internals directly, and accepts no bench-forced "ready" bypass (a
//      prior such mechanism, VCU_switch=0, was identified as an unintended
//      artifact and has been retired).
//
// Relay table (RB+, RB-, SSR, Precharge) decided by the VCU layer:
//   READY:   OFF, OFF, OFF, ON   (precharge; to MOTOR when setpoint > 250 RPM)
//   START:   OFF, OFF, OFF, ON   (unreachable; kept for telemetry names)
//   IDLE:    OFF, ON,  OFF, OFF  (RB- closed, RB+ open; PI off, constant IDLE torque)
//   DYNO:    ON*, ON,  OFF, OFF  (*RB+ closes above 320 V, opens below 300 V)
//   MOTOR:   OFF, OFF, ON,  ON   (SSR is the motor-only power path)
//   FAULT:   OFF, OFF, OFF, OFF  (SSR and RB- open immediately)
// IDLE, DYNO and MOTOR return to READY when HV drops to 250 V or below.
// Precharge is mutually exclusive with RB- (derived from it, never both on),
// and a precharge/RB- swap uses the break-before-make dwell. SSR is mutually
// exclusive with RB- and with RB+.
// ─────────────────────────────────────────────────────────────────────────

namespace {

// ═══════════════════════════════════════════════════════════════════════
// HW LAYER -- physical relay/PWM/LED drive. No knowledge of VCU states.
// ═══════════════════════════════════════════════════════════════════════
namespace hw {

constexpr uint32_t kRelaySwitchDelayMs = 20;  // Break-before-make dwell.
// SSR turns off only at the next mains zero-crossing (up to 10 ms at 50 Hz)
// plus driver delay, so leaving MOTOR needs a longer dwell before RB- closes.
constexpr uint32_t kSsrReleaseDwellMs = 100;

struct RelayCommand {
    bool rbMinusOn = false;
    bool motorPowerOn = false;
    bool rbPlusOn = false;
    bool inverterPrechargeOn = false;
};

// Precharge is mutually exclusive with RB-: it is derived here, not chosen per state.
RelayCommand makeRelayCommand(bool rbMinusOn, bool motorPowerOn, bool rbPlusOn)
{
    RelayCommand cmd;
    cmd.rbMinusOn = rbMinusOn;
    cmd.motorPowerOn = motorPowerOn;
    cmd.rbPlusOn = rbPlusOn;
    cmd.inverterPrechargeOn = !rbMinusOn;
    return cmd;
}

RelayCommand activeRelayCommand{};

// Non-blocking break-before-make state. While a dwell is running,
// activeRelayCommand already reflects the physical "break" (open) step;
// breakBeforeMakeTarget is what gets written once the dwell elapses.
bool breakBeforeMakePending = false;
uint32_t breakBeforeMakeStartMs = 0;
uint32_t breakBeforeMakeDwellMs = kRelaySwitchDelayMs;
RelayCommand breakBeforeMakeTarget{};

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

// Precharge has its own supply and is driven independently of SSR. The SSR
// interlock here is the last line of defence: no caller can energise SSR
// together with RB- or RB+.
void writeRelayPinsImmediate(RelayCommand cmd)
{
    if (cmd.motorPowerOn) {
        cmd.rbMinusOn = false;
        cmd.rbPlusOn = false;
    }
    if (cmd.rbMinusOn) {
        cmd.inverterPrechargeOn = false;
    }
    digitalWrite(MetaSense::Globals::kInverterPrechargePin, cmd.inverterPrechargeOn ? HIGH : LOW);
    digitalWrite(MetaSense::Globals::kMotorPowerPin, cmd.motorPowerOn ? HIGH : LOW);
    digitalWrite(MetaSense::Globals::kRbMinusFetPin, cmd.rbMinusOn ? HIGH : LOW);
    digitalWrite(MetaSense::Globals::kRbPlusRelayPin, cmd.rbPlusOn ? HIGH : LOW);
}

bool relayCommandEquals(const RelayCommand& a, const RelayCommand& b)
{
    return a.rbMinusOn == b.rbMinusOn &&
        a.motorPowerOn == b.motorPowerOn &&
        a.rbPlusOn == b.rbPlusOn &&
        a.inverterPrechargeOn == b.inverterPrechargeOn;
}

// Applies a relay command and enforces the general invariant that SSR is
// mutually exclusive with RB- and RB+. Precharge is an independent supply and
// is not tied to SSR. This is the single point of authority for relay outputs.
//
// Break-before-make (open, wait the relay switch delay, then close) applies
// whenever SSR swaps with RB- or RB+, so the motor path and the grid/dump
// path are never bridged at the same time.
//
// The dwell is non-blocking (millis()-based): this function must be called
// repeatedly (every applyOutputs() tick) so it can notice when the dwell has
// elapsed and complete the "make" step. It never calls delay(), so it can't
// stall the caller's loop (which also drives the PI control loop and other
// time-sensitive work).
void applyRelayCommand(RelayCommand target)
{
    if (target.motorPowerOn) {
        target.rbMinusOn = false;
        target.rbPlusOn = false;
    }

    // A break-before-make dwell is already in flight: the relays are
    // physically mid-transition (open) and must not be re-commanded until
    // the dwell elapses, regardless of what target is requested meanwhile.
    if (breakBeforeMakePending) {
        if (millis() - breakBeforeMakeStartMs < breakBeforeMakeDwellMs) {
            return;
        }
        writeRelayPinsImmediate(breakBeforeMakeTarget);
        activeRelayCommand = breakBeforeMakeTarget;
        breakBeforeMakePending = false;
    }

    if (relayCommandEquals(activeRelayCommand, target)) {
        return;
    }

    const bool rbMinusGoingLow = activeRelayCommand.rbMinusOn && !target.rbMinusOn;
    const bool rbPlusGoingLow = activeRelayCommand.rbPlusOn && !target.rbPlusOn;
    const bool motorPowerGoingLow = activeRelayCommand.motorPowerOn && !target.motorPowerOn;
    const bool rbMinusGoingHigh = !activeRelayCommand.rbMinusOn && target.rbMinusOn;
    const bool rbPlusGoingHigh = !activeRelayCommand.rbPlusOn && target.rbPlusOn;
    const bool motorPowerGoingHigh = !activeRelayCommand.motorPowerOn && target.motorPowerOn;
    const bool prechargeGoingLow = activeRelayCommand.inverterPrechargeOn && !target.inverterPrechargeOn;
    const bool prechargeGoingHigh = !activeRelayCommand.inverterPrechargeOn && target.inverterPrechargeOn;

    // Precharge and RB- are mutually exclusive, so a swap between them is
    // also break-before-make (both open, then the new path closes).
    const bool rbMinusPrechargeSwap = (rbMinusGoingLow && prechargeGoingHigh) ||
                                      (prechargeGoingLow && rbMinusGoingHigh);
    const bool motorPowerCrossover = (motorPowerGoingLow && (rbMinusGoingHigh || rbPlusGoingHigh)) ||
                                     ((rbMinusGoingLow || rbPlusGoingLow) && motorPowerGoingHigh);
    if (motorPowerCrossover || rbMinusPrechargeSwap) {
        RelayCommand breakStep = activeRelayCommand;
        if (rbMinusGoingLow) breakStep.rbMinusOn = false;
        if (rbPlusGoingLow) breakStep.rbPlusOn = false;
        if (motorPowerGoingLow) breakStep.motorPowerOn = false;
        if (prechargeGoingLow) breakStep.inverterPrechargeOn = false;

        writeRelayPinsImmediate(breakStep);
        activeRelayCommand = breakStep;
        breakBeforeMakeTarget = target;
        breakBeforeMakeDwellMs = motorPowerGoingLow ? kSsrReleaseDwellMs : kRelaySwitchDelayMs;
        breakBeforeMakeStartMs = millis();
        breakBeforeMakePending = true;
        return;
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
    (void)configureOutputPin(MetaSense::Globals::kMotorPowerPin);
    (void)configureOutputPin(MetaSense::Globals::kRbPlusRelayPin);
    (void)configureOutputPin(MetaSense::Globals::kInverterPrechargePin);

    activeRelayCommand = RelayCommand{};
    breakBeforeMakePending = false;
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

bool isMotorPowerActive()
{
    return digitalRead(MetaSense::Globals::kMotorPowerPin) == HIGH;
}

bool isInverterPrechargeActive()
{
    return digitalRead(MetaSense::Globals::kInverterPrechargePin) == HIGH;
}

} // namespace hw

// ═══════════════════════════════════════════════════════════════════════
// VCU LOGIC -- state machine, transitions, precharge sequencing. Only
// talks to the `hw` namespace above via its command-based API.
// ═══════════════════════════════════════════════════════════════════════

enum class OutputState {
    READY,
    START,
    IDLE,
    MOTOR,
    DYNO,
    FAULT
};

const char* outputStateName(OutputState s)
{
    switch (s) {
    case OutputState::READY: return "READY";
    case OutputState::START: return "START";
    case OutputState::IDLE:  return "IDLE";
    case OutputState::MOTOR: return "MOTOR";
    case OutputState::DYNO:  return "DYNO";
    case OutputState::FAULT: return "FAULT";
    }
    return "READY";
}

OutputState state = OutputState::READY;

// --- Tunables from the agreed spec ---
constexpr uint32_t kPrechargeDurationMs = 2000;  // Precharge attempt length; a failed attempt is retried after this.
constexpr uint8_t  kPrechargeMaxRetries = 3;     // Retries after the first failed attempt; then FAULT.
constexpr float kPrechargeHvReadyV = 300.0f;     // HV must reach this to succeed.
constexpr float kPrechargeKeepHvV = 250.0f;      // IDLE/DYNO/MOTOR: at or below this, return to READY.
// Setpoint hysteresis H = 100 RPM around the 150 RPM centre. A setpoint above
// 150+H enters MOTOR from READY/IDLE; below 150-H it leaves MOTOR.
constexpr float kSetpointCentreRpm = 150.0f;
constexpr float kSetpointHystRpm = 100.0f;
constexpr float kMotorEnterSetpointRpm = kSetpointCentreRpm + kSetpointHystRpm;  // 250
constexpr float kMotorExitSetpointRpm = kSetpointCentreRpm - kSetpointHystRpm;   // 50
// RPM hysteresis H. IDLE/DYNO transitions compare the setpoint (not engine RPM)
// against idleThresholdRpm +/- H, since engine RPM overshoots under PI control.
// MOTOR enters DYNO above engine setpoint + H.
constexpr float kEngineHystRpm = 100.0f;
// MOTOR enters DYNO only above this absolute engine RPM as well as setpoint+H.
constexpr float kMotorToDynoMinEngineRpm = 3500.0f;
// IDLE switches from precharge to RB- above Settings::idleThresholdRpm.
// Allowed headroom above motorModeMaxRpm before the setpoint is clamped for state decisions.
constexpr float kMotorSetpointMarginRpm = 1000.0f;
// RB+ (grid inverter feed) closes only in DYNO when HV rises above
// kGridRbPlusCloseHvV, and opens when HV falls below kGridRbPlusOpenHvV.
// Between the two levels it keeps its previous state.
constexpr float kGridRbPlusCloseHvV = 320.0f;
constexpr float kGridRbPlusOpenHvV = 300.0f;
bool gridRbPlusClosed = false;
// IDLE RB- state. Switches on above idleThresholdRpm + H and off below
// idleThresholdRpm - H (setpoint); holds its previous value inside the band.
bool idleRbMinusOn = false;
// --- Precharge sequencing state ---
uint32_t prechargeStartMs = 0;
uint8_t prechargeAttempt = 0;
// Boot gate: READY may not go operative until the RPM setpoint has been 0 once
// since power-up. A reboot (including FAULT_RESET, which restarts) clears it.
bool bootSetpointGateOpen = false;

// stop() may be called from the network task while update() runs on the
// control task. stop() only posts this request; update() applies it, so the
// relay FSM has a single writer.
portMUX_TYPE stopRequestMux = portMUX_INITIALIZER_UNLOCKED;
bool stopRequested = false;

bool takeStopRequest()
{
    bool requested = false;
    portENTER_CRITICAL(&stopRequestMux);
    requested = stopRequested;
    stopRequested = false;
    portEXIT_CRITICAL(&stopRequestMux);
    return requested;
}

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

    case OutputState::READY:
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
    case OutputState::READY:
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
                  float rpmSetpoint,
                  float hvVoltage)
{
    hw::writeThrottleDutyPwm(engineThrottlePercent);

    hw::RelayCommand cmd{};

    if (hwState != OutputState::DYNO) {
        gridRbPlusClosed = false;
    }
    if (hwState != OutputState::IDLE) {
        idleRbMinusOn = false;
    }

    switch (hwState) {
    case OutputState::READY:
    case OutputState::START:
        // Precharge ON (RB- is OFF), RB+/SSR OFF.
        cmd = hw::makeRelayCommand(false, false, false);
        break;

    case OutputState::IDLE: {
        const float idleThreshold = MetaSense::Settings::idleThresholdRpm;
        if (rpmSetpoint > idleThreshold + kEngineHystRpm) {
            idleRbMinusOn = true;
        } else if (rpmSetpoint < idleThreshold - kEngineHystRpm) {
            idleRbMinusOn = false;
        }
        // RB- ON with precharge OFF, or precharge ON with RB- OFF.
        if (idleRbMinusOn) {
            cmd = hw::makeRelayCommand(true, false, false);
            cmd.inverterPrechargeOn = false;
        } else {
            cmd = hw::makeRelayCommand(false, false, false);
        }
        break;
    }

    case OutputState::DYNO:
        // RB- ON, SSR OFF. RB+ closes above kGridRbPlusCloseHvV
        // and opens below kGridRbPlusOpenHvV, connecting DYNO power to the grid.
        if (hvVoltage > kGridRbPlusCloseHvV) {
            gridRbPlusClosed = true;
        } else if (hvVoltage < kGridRbPlusOpenHvV) {
            gridRbPlusClosed = false;
        }
        cmd = hw::makeRelayCommand(true, false, gridRbPlusClosed);
        break;

    case OutputState::MOTOR:
        // SSR is the motor-only power path. RB+/RB- OFF, so precharge is ON.
        cmd = hw::makeRelayCommand(false, true, false);
        break;

    case OutputState::FAULT:
        // SSR and RB- open immediately, RB+ opens with them, and precharge
        // is released.
        cmd = hw::makeRelayCommand(false, false, false);
        cmd.inverterPrechargeOn = false;
        break;

    default:
        cmd = hw::makeRelayCommand(false, false, false);
        cmd.inverterPrechargeOn = false;
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

    state = OutputState::READY;
    prechargeStartMs = 0;
    prechargeAttempt = 0;
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
            float requestedRpmSetpoint,
            float primaryBrakePercent,
            float engineRpm,
            float hvVoltage,
            bool inverterReady,
            bool canFeedbackFresh)
{
    const uint32_t now = millis();
    // Setpoint is limited to motorModeMaxRpm + kMotorSetpointMarginRpm for
    // state decisions, so a setpoint above the configured max cannot hold
    // the machine in MOTOR or DYNO.
    const float maxSetpointRpm = MetaSense::Settings::motorModeMaxRpm + kMotorSetpointMarginRpm;
    const float rpmSetpoint = (requestedRpmSetpoint > maxSetpointRpm) ? maxSetpointRpm : requestedRpmSetpoint;

    // Stop is applied here, on the control task, so it never races the
    // relay FSM. A latched FAULT survives a recording stop; otherwise the
    // IDLE pattern would re-energise RB- on the dyno.
    if (takeStopRequest()) {
        writeThrottle(0.0f);
        if (state != OutputState::FAULT) {
            state = OutputState::IDLE;
        }
        applyOutputs(state, 0.0f, 0.0f, 0.0f, hvVoltage);
        return;
    }

    // FAULT is reachable from IDLE/MOTOR/DYNO when the inverter is not ready
    // or live 0x1DA feedback is lost. It is NOT reachable this way from START
    // -- during precharge, only the HV-not-reached-in-time condition matters.
    const bool faultConditionActive = !inverterReady || !canFeedbackFresh;
    if (faultConditionActive &&
        (state == OutputState::IDLE || state == OutputState::MOTOR || state == OutputState::DYNO)) {
        state = OutputState::FAULT;
    }

    // Bus sag (e.g. 0 RPM with no generator EMF) returns to READY, which
    // re-attempts precharge every kPrechargeDurationMs until HV recovers.
    if (hvVoltage <= kPrechargeKeepHvV &&
        (state == OutputState::IDLE || state == OutputState::MOTOR || state == OutputState::DYNO)) {
        state = OutputState::READY;
        prechargeStartMs = 0;
        prechargeAttempt = 0;
    }

    if (!bootSetpointGateOpen && rpmSetpoint <= 0.0f) {
        bootSetpointGateOpen = true;
    }

    switch (state) {
    case OutputState::READY: {
        // Precharge runs here. Each attempt lasts kPrechargeDurationMs. At the
        // end of an attempt, HV below kPrechargeHvReadyV is a failure: the
        // attempt is retried up to kPrechargeMaxRetries times, then FAULT.
        // With no RPM demand the machine holds in READY, so HV is not discharged
        // by bouncing between READY and IDLE. After boot the boot gate must also
        // be open to leave READY.
        if (prechargeStartMs == 0) {
            prechargeStartMs = now;
        }
        const uint32_t elapsedMs = now - prechargeStartMs;
        if (elapsedMs < kPrechargeDurationMs) {
            break;
        }
        if (hvVoltage < kPrechargeHvReadyV) {
            if (prechargeAttempt < kPrechargeMaxRetries) {
                ++prechargeAttempt;
                prechargeStartMs = now;
            } else {
                state = OutputState::FAULT;
            }
            break;
        }
        if (bootSetpointGateOpen && canFeedbackFresh &&
            (rpmSetpoint > kMotorEnterSetpointRpm)) {
            state = OutputState::MOTOR;
        }
        break;
    }

    case OutputState::START:
        // No transition enters START. Map it to READY so it cannot hold a
        // stale precharge timer.
        state = OutputState::READY;
        break;

    case OutputState::IDLE:
        // Over-speed is braked first, so DYNO takes priority over MOTOR.
        if (rpmSetpoint > MetaSense::Settings::idleThresholdRpm + kEngineHystRpm) {
            state = OutputState::DYNO;
        } else if (rpmSetpoint > kMotorEnterSetpointRpm) {
            state = OutputState::MOTOR;
        }
        break;

    case OutputState::MOTOR:
        if (rpmSetpoint < kMotorExitSetpointRpm) {
            state = OutputState::IDLE;
        } else if (engineRpm > kMotorToDynoMinEngineRpm &&
                   engineRpm > rpmSetpoint + kEngineHystRpm) {
            state = OutputState::DYNO;
        }
        break;

    case OutputState::DYNO:
        // DYNO -> MOTOR passes through IDLE, which re-evaluates the setpoint.
        if (rpmSetpoint < MetaSense::Settings::idleThresholdRpm - kEngineHystRpm) {
            state = OutputState::IDLE;
        }
        break;

    case OutputState::FAULT:
        // Latched: no automatic exit. The operator reset restarts the controller,
        // so FAULT is only left through a reboot.
        break;
    }

    applyOutputs(state, engineThrottlePercent, primaryBrakePercent, rpmSetpoint, hvVoltage);
}

bool isMotorState()
{
    return state == OutputState::MOTOR;
}

bool isIdleState()
{
    return state == OutputState::IDLE;
}

bool isDynoState()
{
    return state == OutputState::DYNO;
}

bool isFaultState()
{
    return state == OutputState::FAULT;
}

void stop()
{
    portENTER_CRITICAL(&stopRequestMux);
    stopRequested = true;
    portEXIT_CRITICAL(&stopRequestMux);
}

const char* stateName()
{
    return outputStateName(state);
}

bool isRbPlusActive()
{
    return hw::isRbPlusActive();
}

bool isRbMinusActive()
{
    return hw::isRbMinusActive();
}

bool isRbMinusCommandedActive()
{
    return hw::activeRelayCommand.rbMinusOn;
}

bool isMotorPowerActive()
{
    return hw::isMotorPowerActive();
}

bool isMotorPowerCommandedActive()
{
    return hw::activeRelayCommand.motorPowerOn;
}

bool isInverterPrechargeActive()
{
    return hw::isInverterPrechargeActive();
}

bool isInverterPrechargeCommandedActive()
{
    return hw::activeRelayCommand.inverterPrechargeOn;
}

bool isPrechargeSucceeded()
{
    return (state != OutputState::READY) && (state != OutputState::START) && (state != OutputState::FAULT);
}

} // namespace MetaSense::HardwareOutputStateMachine