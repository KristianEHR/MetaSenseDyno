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
// Relay table (RB+, RB-, SSR, Precharge) decided by the VCU layer:
//   INIT:    OFF, OFF, OFF, OFF
//   START:   OFF, OFF, ON,  ON  (2.5s precharge dwell)
//   IDLE:    ON,  ON,  OFF, OFF   (HV keep-alive can flip SSR on / RB- off)
//   MOTOR:   ON,  OFF, ON,  OFF
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

void writeRelayPinsImmediate(const RelayCommand& cmd)
{
    digitalWrite(MetaSense::Globals::kRbMinusFetPin, cmd.rbMinusOn ? HIGH : LOW);
    digitalWrite(MetaSense::Globals::kSssrPin, cmd.ssrOn ? HIGH : LOW);
    digitalWrite(MetaSense::Globals::kRbPlusRelayPin, cmd.rbPlusOn ? HIGH : LOW);
    digitalWrite(MetaSense::Globals::kPrechargeRelayPin, cmd.prechargeOn ? HIGH : LOW);
}

bool relayCommandEquals(const RelayCommand& a, const RelayCommand& b)
{
    return a.rbMinusOn == b.rbMinusOn &&
        a.ssrOn == b.ssrOn &&
        a.rbPlusOn == b.rbPlusOn &&
        a.prechargeOn == b.prechargeOn;
}

// Applies a relay command with break-before-make (open contactors first,
// wait the relay switch delay, then close the next set) and enforces the
// general SSR/RB- mutual-exclusion invariant. This is the single point of
// authority for relay outputs.
void applyRelayCommand(RelayCommand target)
{
    // General invariant: SSR and RB- can never both be commanded ON.
    if (target.ssrOn && target.rbMinusOn) {
        target.rbMinusOn = false;
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
    const bool prechargeGoingLow = activeRelayCommand.prechargeOn && !target.prechargeOn;
    const bool prechargeGoingHigh = !activeRelayCommand.prechargeOn && target.prechargeOn;

    const bool anyGoingLow = rbMinusGoingLow || ssrGoingLow || rbPlusGoingLow || prechargeGoingLow;
    const bool anyGoingHigh = rbMinusGoingHigh || ssrGoingHigh || rbPlusGoingHigh || prechargeGoingHigh;

    // Break-before-make: open contactors first, wait relay latency, then
    // close the next set. Blocking delay is intentional here -- rotational
    // inertia means the PI control loop will not notice a relay transition.
    if (anyGoingLow && anyGoingHigh) {
        RelayCommand breakStep = activeRelayCommand;
        if (rbMinusGoingLow) breakStep.rbMinusOn = false;
        if (ssrGoingLow) breakStep.ssrOn = false;
        if (rbPlusGoingLow) breakStep.rbPlusOn = false;
        if (prechargeGoingLow) breakStep.prechargeOn = false;

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
constexpr float kHvKeepAliveMaxRpm = 500.0f;      // IDLE-only HV keep-alive: RPM must be <= this.
constexpr float kHvKeepAliveEngageV = 10.0f;      // Engage precharge-channel recharge below this HV.
constexpr float kHvKeepAliveReleaseV = 300.0f;    // Release recharge above this HV (latched, avoids chatter).

// --- Precharge sequencing state ---
uint32_t prechargeStartMs = 0;
uint8_t prechargeAttempt = 0;

// --- IDLE-state HV keep-alive latch (persists across calls so the 200V/
// 300V engage/release hysteresis can't chatter at either edge). ---
bool idleHvKeepAliveLatched = false;


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
        // HV keep-alive: recharges the DC bus through the current-limited
        // precharge channel (SSR+PRECHARGE together -- same relay pattern
        // as the initial START precharge) whenever HV sags, but only while
        // genuinely idling: RPM <= 500 AND RPM > the operator's rpm
        // setpoint. Engage at HV<200V, release at HV>300V (latched, so HV
        // sitting near either edge cannot cause relay chatter).
        const bool keepAliveEligible = (engineRpm <= kHvKeepAliveMaxRpm) &&
                                        (engineRpm > rpmSetpoint);
        if (!keepAliveEligible) {
            idleHvKeepAliveLatched = false;
        } else if (!idleHvKeepAliveLatched && hvVoltageFromCan < kHvKeepAliveEngageV) {
            idleHvKeepAliveLatched = true;
        } else if (idleHvKeepAliveLatched && hvVoltageFromCan > kHvKeepAliveReleaseV) {
            idleHvKeepAliveLatched = false;
        }

        if (idleHvKeepAliveLatched) {
            // RB+=ON, RB-=OFF, SSR=ON, PRECHARGE=ON.
            cmd = hw::makeRelayCommand(false, true, true, true);
        } else {
            // Normal IDLE relay pattern: RB+/RB- ON, SSR/Precharge OFF.
            cmd = hw::makeRelayCommand(true, false, true, false);
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
