#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/gpio.h>

#include "HardwareOutputStateMachine.h"
#include "globals.h"

namespace {

#ifndef METASENSE_RBPLUS_RELAY_ENABLED
// Default off: release RB+ GPIO from firmware control.
#define METASENSE_RBPLUS_RELAY_ENABLED 0
#endif

#ifndef METASENSE_PRECHARGE_TEST_ACCEPT_TIMEOUT
// Allow the precharge dwell to complete even if the inverter does not publish
// a READY bit yet. That keeps the output state machine from stalling in INIT.
#define METASENSE_PRECHARGE_TEST_ACCEPT_TIMEOUT 1
#endif

enum class OutputState {
    INIT,
    START,
    IDLE,
    MOTOR,
    DYNO
};

OutputState state = OutputState::INIT;
OutputState pendingState = OutputState::INIT;
uint32_t pendingStateSinceMs = 0;
constexpr uint32_t kRelaySwitchDelayMs = 20;
constexpr uint32_t kStateDebounceMs = 50;
constexpr uint32_t kInitPrereqStableMs = 500;
constexpr uint32_t kInitPrechargeMinMs = 2500;
constexpr uint32_t kStartSetpointWaitMs = 15000;
constexpr float kInitPrechargeHvReadyV = 300.0f;
constexpr float kStartFallbackHvLowV = 200.0f;
constexpr float kIdleSetpointZeroThresholdRpm = 0.0f;
constexpr float kDynoModeEntryRpm = 500.0f;
constexpr float kIdleEntryMaxRpm = 500.0f;
constexpr float kRpmHysteresisBandRpm = 100.0f;
bool inverterStatusInitialized = false;
bool hvArmedLatched = false;
bool initPrechargeActive = false;
bool initPrechargeCompleted = false;
bool initPrechargeSucceeded = false;
bool initPrechargeFailed = false;
bool prestartWarnPrinted = false;
uint32_t initPrechargeStartMs = 0;
uint32_t initReadySinceMs = 0;
uint32_t startSetpointWaitStartMs = 0;
const char* startGateReasonText = "boot";
bool startResetRequestPending = false;
bool vcuRelayOverrideEnabled = false;
bool vcuRbPlusCommand = false;
bool vcuPrechargeCommand = false;
bool vcuSsrCommand = false;
bool vcuRbMinusCommand = false;

struct RelayCommand {
    bool rbMinusOn = false;
    bool ssrOn = false;
    bool rbPlusOn = false;
    bool prechargeOn = false;
};

RelayCommand activeRelayCommand{};
bool relayInvariantMismatchLatched = false;
uint32_t relayInvariantLastLogMs = 0;
uint32_t relayInvariantMismatchCount = 0;
constexpr uint32_t kRelayInvariantLogPeriodMs = 1000;

Adafruit_NeoPixel strip(MetaSense::Globals::kOnboardLedCount,
                        MetaSense::Globals::kOnboardLedPin,
                        NEO_GRB + NEO_KHZ800);

const char* outputStateName(OutputState hwState)
{
    switch (hwState) {
    case OutputState::INIT:
        return "INIT";
    case OutputState::START:
        return "START";
    case OutputState::IDLE:
        return "IDLE";
    case OutputState::MOTOR:
        return "MOTOR";
    case OutputState::DYNO:
        return "DYNO";
    }

    return "IDLE";
}

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
    int pwm = static_cast<int>(percent * static_cast<float>(maxPwm) / 100.0f);
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
#if METASENSE_RBPLUS_RELAY_ENABLED
    digitalWrite(MetaSense::Globals::kRbPlusRelayPin, cmd.rbPlusOn ? HIGH : LOW);
#endif
    digitalWrite(MetaSense::Globals::kPrechargeRelayPin, cmd.prechargeOn ? HIGH : LOW);
}

bool relayCommandEquals(const RelayCommand& a, const RelayCommand& b)
{
    return a.rbMinusOn == b.rbMinusOn &&
        a.ssrOn == b.ssrOn &&
        a.rbPlusOn == b.rbPlusOn &&
        a.prechargeOn == b.prechargeOn;
}

void logRelayInvariant(OutputState hwState,
                       const RelayCommand& expected,
                       const RelayCommand& actual,
                       uint32_t now)
{
    ++relayInvariantMismatchCount;
    Serial.printf("[HWSM][GUARD] mismatch=%lu state=%s exp(RB-=%d SSR=%d RB+=%d PRE=%d) act(RB-=%d SSR=%d RB+=%d PRE=%d)\\n",
                  static_cast<unsigned long>(relayInvariantMismatchCount),
                  outputStateName(hwState),
                  expected.rbMinusOn ? 1 : 0,
                  expected.ssrOn ? 1 : 0,
                  expected.rbPlusOn ? 1 : 0,
                  expected.prechargeOn ? 1 : 0,
                  actual.rbMinusOn ? 1 : 0,
                  actual.ssrOn ? 1 : 0,
                  actual.rbPlusOn ? 1 : 0,
                  actual.prechargeOn ? 1 : 0);
    Serial0.printf("[HWSM][GUARD] mismatch=%lu state=%s exp(RB-=%d SSR=%d RB+=%d PRE=%d) act(RB-=%d SSR=%d RB+=%d PRE=%d)\\n",
                   static_cast<unsigned long>(relayInvariantMismatchCount),
                   outputStateName(hwState),
                   expected.rbMinusOn ? 1 : 0,
                   expected.ssrOn ? 1 : 0,
                   expected.rbPlusOn ? 1 : 0,
                   expected.prechargeOn ? 1 : 0,
                   actual.rbMinusOn ? 1 : 0,
                   actual.ssrOn ? 1 : 0,
                   actual.rbPlusOn ? 1 : 0,
                   actual.prechargeOn ? 1 : 0);
    relayInvariantLastLogMs = now;
}

void checkRelayInvariant(OutputState hwState, uint32_t now)
{
    RelayCommand expected{};
    bool hasExpectation = false;

    switch (hwState) {
    case OutputState::IDLE:
        // IDLE contract: brake side closed, SSR open, RB+ follows precharge success latch.
        expected.rbMinusOn = true;
        expected.ssrOn = false;
        expected.rbPlusOn = initPrechargeSucceeded;
        expected.prechargeOn = false;
        hasExpectation = true;
        break;
    case OutputState::MOTOR:
        // MOTOR contract: SSR closed, RB- open, RB+ follows precharge success latch.
        expected.rbMinusOn = false;
        expected.ssrOn = true;
        expected.rbPlusOn = initPrechargeSucceeded;
        expected.prechargeOn = false;
        hasExpectation = true;
        break;
    default:
        relayInvariantMismatchLatched = false;
        return;
    }

    if (!hasExpectation) {
        return;
    }

    const bool mismatch = !relayCommandEquals(activeRelayCommand, expected);
    if (!mismatch) {
        relayInvariantMismatchLatched = false;
        return;
    }

    const bool logNow = !relayInvariantMismatchLatched ||
        (now - relayInvariantLastLogMs) >= kRelayInvariantLogPeriodMs;
    if (logNow) {
        logRelayInvariant(hwState, expected, activeRelayCommand, now);
    }
    relayInvariantMismatchLatched = true;
}

void applyRelayCommand(const RelayCommand& target)
{
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

    // Enforce break-before-make: open contactors first, wait relay latency, then close the next set.
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

void setRelayOutputs(bool rbMinusOn, bool sssrOn, bool rbPlusOn = false, bool prechargeOn = false)
{
    RelayCommand cmd{};
    cmd.rbMinusOn = vcuRelayOverrideEnabled ? vcuRbMinusCommand : rbMinusOn;

    const bool requestedSsrOn = vcuRelayOverrideEnabled ? vcuSsrCommand : sssrOn;
    cmd.ssrOn = requestedSsrOn;

    // New HV relay pins are owned by VCU path when override is enabled.
    cmd.rbPlusOn = vcuRelayOverrideEnabled ? vcuRbPlusCommand : rbPlusOn;
    cmd.prechargeOn = vcuRelayOverrideEnabled ? vcuPrechargeCommand : prechargeOn;

    // Precharge ownership is centralized to the START precharge sequence only.
    if (state != OutputState::START || !initPrechargeActive) {
        cmd.prechargeOn = false;
    }

    // Safety interlock: precharge path is only valid while RB+ is open.
    if (cmd.prechargeOn) {
        cmd.rbPlusOn = false;
    }

    applyRelayCommand(cmd);
}

void writePrimaryBrakeSplit(OutputState hwState, float signedPercent)
{
    signedPercent = constrain(signedPercent, -100.0f, 100.0f);

    switch (hwState) {
    case OutputState::INIT:
        writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel, 0.0f);
        writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel, 0.0f);
        break;
    case OutputState::START:
        writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel, 0.0f);
        writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel, 0.0f);
        break;
    case OutputState::MOTOR:
        writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel,
                     signedPercent > 0.0f ? signedPercent : 0.0f);
        writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel, 0.0f);
        break;

    case OutputState::DYNO:
        writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel, 0.0f);
        writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel,
                     signedPercent < 0.0f ? -signedPercent : 0.0f);
        break;

    case OutputState::IDLE:
        writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel, 0.0f);
        writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel, 0.0f);
        break;
    }
}

void setStateLed(OutputState hwState)
{
    uint32_t color = 0;

    switch (hwState) {
    case OutputState::INIT:
        color = strip.Color(255, 255, 255);
        break;
    case OutputState::START:
        color = strip.Color(255, 200, 0);
        break;
    case OutputState::IDLE:
        color = strip.Color(0, 0, 255);
        break;
    case OutputState::MOTOR:
        color = strip.Color(255, 0, 0);
        break;
    case OutputState::DYNO:
        color = strip.Color(0, 255, 0);
        break;
    }

    strip.setPixelColor(0, color);
    strip.show();
}

void updateStartPrechargeSequence(float setPoint,
                                  float inverterHvVoltage,
                                  bool inverterFault,
                                  uint32_t now)
{
    if (initPrechargeFailed) {
        initPrechargeActive = false;
        return;
    }

    if (initPrechargeCompleted) {
        initPrechargeActive = false;
        return;
    }

    // Enforce START precharge only at zero setpoint.
    const bool startCondition = setPoint <= kIdleSetpointZeroThresholdRpm;
    if (!startCondition) {
        initPrechargeActive = false;
        return;
    }

    if (!initPrechargeActive) {
        initPrechargeActive = true;
        initPrechargeStartMs = now;
        return;
    }

    const uint32_t elapsedMs = now - initPrechargeStartMs;
    if (elapsedMs >= kInitPrechargeMinMs) {
        initPrechargeActive = false;
        initPrechargeCompleted = true;

        const bool hvReady = inverterHvVoltage >= kInitPrechargeHvReadyV;
        const bool faultFree = !inverterFault;
        initPrechargeSucceeded = hvReady && faultFree;
        initPrechargeFailed = !initPrechargeSucceeded;

        if (!initPrechargeSucceeded && !prestartWarnPrinted) {
            Serial.printf("[HWSM][PRESTART_WARN] START precharge complete but not armed: hv=%.1fV fault=%d (need hv>=%.1f and fault=0)\n",
                          inverterHvVoltage,
                          inverterFault ? 1 : 0,
                          kInitPrechargeHvReadyV);
            Serial0.printf("[HWSM][PRESTART_WARN] START precharge complete but not armed: hv=%.1fV fault=%d (need hv>=%.1f and fault=0)\n",
                           inverterHvVoltage,
                           inverterFault ? 1 : 0,
                           kInitPrechargeHvReadyV);
            prestartWarnPrinted = true;
        }
    }
}

void applyOutputs(OutputState nextState,
                  OutputState prevState,
                  float engineThrottlePercent,
                  float primaryBrakePercent,
                  bool inverterReady,
                  bool inverterFault)
{
    (void)prevState;
    writeThrottleDutyPwm(engineThrottlePercent);

    if (initPrechargeFailed) {
        setRelayOutputs(false, false, false, false);
        writePrimaryBrakeSplit(OutputState::START, 0.0f);
        setStateLed(OutputState::START);
        return;
    }

    hvArmedLatched = inverterReady;

    // RB+ is owned by the init/precharge sequence only and remains latched until fault or power-off.
    const bool rbPlusHold = initPrechargeSucceeded;
    const bool prechargeActive = initPrechargeActive;

    switch (nextState) {
    case OutputState::INIT:
        // INIT contract: firmware/hardware bring-up complete, keep relays OFF.
        setRelayOutputs(false, false, false, false);
        writePrimaryBrakeSplit(OutputState::INIT, 0.0f);
        break;

    case OutputState::START:
        // START contract: all relays OFF except timed precharge window.
        setRelayOutputs(false, prechargeActive, false, prechargeActive);
        writePrimaryBrakeSplit(OutputState::START, 0.0f);
        break;

    case OutputState::IDLE:
        setRelayOutputs(true, false, rbPlusHold);
        writePrimaryBrakeSplit(OutputState::IDLE, primaryBrakePercent);
        break;

    case OutputState::MOTOR:
        setRelayOutputs(false, true, rbPlusHold, prechargeActive);
        writePrimaryBrakeSplit(OutputState::MOTOR, primaryBrakePercent);
        break;

    case OutputState::DYNO:
        setRelayOutputs(true, false, rbPlusHold);
        writePrimaryBrakeSplit(OutputState::DYNO, primaryBrakePercent);
        break;
    }

    setStateLed(nextState);
}

} // anonymous namespace


namespace MetaSense::HardwareOutputStateMachine {

void begin()
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

    const bool rbMinusValid = configureOutputPin(MetaSense::Globals::kRbMinusFetPin);
    const bool ssrValid = configureOutputPin(MetaSense::Globals::kSssrPin);
#if METASENSE_RBPLUS_RELAY_ENABLED
    const bool rbPlusValid = configureOutputPin(MetaSense::Globals::kRbPlusRelayPin);
#else
    const bool rbPlusValid = false;
#endif
    const bool prechargeValid = configureOutputPin(MetaSense::Globals::kPrechargeRelayPin);
    (void)rbMinusValid;
    (void)ssrValid;
    (void)rbPlusValid;
    (void)prechargeValid;

    activeRelayCommand = RelayCommand{};
    setRelayOutputs(false, false);
    state = OutputState::INIT;
    pendingState = state;
    pendingStateSinceMs = millis();
    initPrechargeActive = false;
    initPrechargeCompleted = false;
    initPrechargeSucceeded = false;
    initPrechargeFailed = false;
    prestartWarnPrinted = false;
    initPrechargeStartMs = 0;
    setStateLed(state);
}

void writeThrottle(float percent)
{
    writeThrottleDutyPwm(percent);
}

void writeBrake(float percent)
{
    writeActuatorChannel(MetaSense::Globals::kBrakePwmChannel, percent);
}

void writeDynoThrottle(float percent)
{
    writeActuatorChannel(MetaSense::Globals::kDynoThrottlePwmChannel, percent);
}

void update(float engineThrottlePercent,
            float setPoint,
            float rpm,
            float primaryBrakePercent,
            float inverterHvVoltage,
            bool inverterStatusReady,
            bool inverterReady,
            bool inverterFault,
            bool canTelemetryReady,
            bool sensorsReady)
{
    const OutputState prevState = state;
    OutputState candidateState = state;
    const bool idleSetpointZero = setPoint <= kIdleSetpointZeroThresholdRpm;
    // Bidirectional RPM hysteresis: use separate entry/exit thresholds to avoid chatter
    // when crossing IDLE and DYNO boundaries in either direction.
    const float idleEnterRpm = kIdleEntryMaxRpm - kRpmHysteresisBandRpm;
    const float idleExitRpm = kIdleEntryMaxRpm + kRpmHysteresisBandRpm;
    const bool idleRpmCondition = (state == OutputState::IDLE)
        ? (rpm <= idleExitRpm)
        : (rpm <= idleEnterRpm);
    const bool idleCondition = idleSetpointZero && idleRpmCondition;

    const float dynoEnterRpm = kDynoModeEntryRpm + kRpmHysteresisBandRpm;
    const float dynoExitRpm = kDynoModeEntryRpm - kRpmHysteresisBandRpm;
    const bool dynoRpmCondition = (state == OutputState::DYNO)
        ? (rpm >= dynoExitRpm)
        : (rpm >= dynoEnterRpm);
    const bool dynoCondition = dynoRpmCondition;
    const bool motorSetpointCondition = setPoint > kIdleSetpointZeroThresholdRpm;

    if (inverterStatusReady) {
        inverterStatusInitialized = true;
    }

    const uint32_t now = millis();
    bool forceStartByRequest = false;
    if (startResetRequestPending) {
        startResetRequestPending = false;
        forceStartByRequest = true;

        // Start-button reset: clear START sequencing latches and retry from START.
        initPrechargeActive = false;
        initPrechargeCompleted = false;
        initPrechargeSucceeded = false;
        initPrechargeFailed = false;
        prestartWarnPrinted = false;
        initPrechargeStartMs = 0;
        startSetpointWaitStartMs = 0;
    }

    const bool initPrereqNow = canTelemetryReady && sensorsReady;
    if (initPrereqNow) {
        if (initReadySinceMs == 0U) {
            initReadySinceMs = now;
        }
    } else {
        initReadySinceMs = 0U;
    }
    const bool initPrereqStable = (initReadySinceMs != 0U) &&
                                  ((now - initReadySinceMs) >= kInitPrereqStableMs);

    const bool hvTooLowForRun = inverterHvVoltage < kStartFallbackHvLowV;
    if (hvTooLowForRun) {
        initPrechargeActive = false;
        startSetpointWaitStartMs = 0;
    }
    if (hvTooLowForRun && state != OutputState::START) {
        initPrechargeActive = false;
        initPrechargeCompleted = false;
        initPrechargeSucceeded = false;
        initPrechargeFailed = false;
        prestartWarnPrinted = false;
        initPrechargeStartMs = 0;
        startSetpointWaitStartMs = 0;
    }

    if (state == OutputState::START && !hvTooLowForRun) {
        updateStartPrechargeSequence(setPoint, inverterHvVoltage, inverterFault, now);
    } else {
        initPrechargeActive = false;
    }

    if (forceStartByRequest) {
        candidateState = OutputState::START;
    } else if (state == OutputState::INIT) {
        // INIT is the full firmware/hardware bring-up phase.
        // Auto-promote to START once prerequisites are stably ready.
        // A start-button request can still force START immediately.
        candidateState = initPrereqStable ? OutputState::START : OutputState::INIT;
    } else if (hvTooLowForRun || inverterFault) {
        candidateState = OutputState::START;
    } else if (state == OutputState::START && !initPrechargeSucceeded) {
        candidateState = OutputState::START;
    } else if (state == OutputState::START) {
        // START exit policy: after precharge gate passes, allow up to 15s
        // for setpoint > 0 to move into MOTOR; otherwise fall back to IDLE.
        if (motorSetpointCondition) {
            startSetpointWaitStartMs = 0;
            candidateState = OutputState::MOTOR;
        } else {
            if (startSetpointWaitStartMs == 0U) {
                startSetpointWaitStartMs = now;
            }
            const bool startWaitExpired = (now - startSetpointWaitStartMs) >= kStartSetpointWaitMs;
            candidateState = startWaitExpired ? OutputState::IDLE : OutputState::START;
        }
    } else if (dynoCondition) {
        candidateState = OutputState::DYNO;
    } else if (motorSetpointCondition) {
        candidateState = OutputState::MOTOR;
    } else if (idleCondition) {
        candidateState = OutputState::IDLE;
    } else {
        candidateState = OutputState::IDLE;
    }

    // Operator-facing reason why START is currently held, or why run path is open.
    if (forceStartByRequest) {
        startGateReasonText = "start_button";
    } else if (state == OutputState::INIT) {
        startGateReasonText = initPrereqStable ? "init_to_start" : "init_wait_prereq";
    } else if (hvTooLowForRun) {
        startGateReasonText = "hv_low";
    } else if (inverterFault) {
        startGateReasonText = "fault";
    } else if (state == OutputState::START && !initPrechargeSucceeded) {
        if (initPrechargeFailed) {
            startGateReasonText = "precharge_failed";
        } else if (setPoint > kIdleSetpointZeroThresholdRpm) {
            startGateReasonText = "wait_setpoint_zero";
        } else if (initPrechargeActive) {
            startGateReasonText = "precharge_active";
        } else {
            startGateReasonText = "precharge_pending";
        }
    } else if (state == OutputState::START) {
        if (motorSetpointCondition) {
            startGateReasonText = "start_exit_motor";
        } else if (startSetpointWaitStartMs == 0U) {
            startGateReasonText = "wait_setpoint_positive";
        } else if ((now - startSetpointWaitStartMs) >= kStartSetpointWaitMs) {
            startGateReasonText = "start_timeout_idle";
        } else {
            startGateReasonText = "wait_setpoint_positive";
        }
    } else {
        startGateReasonText = "open";
    }

    if (candidateState != pendingState) {
        pendingState = candidateState;
        pendingStateSinceMs = now;
    }

    if (pendingState != state && (now - pendingStateSinceMs) >= kStateDebounceMs) {
        state = pendingState;
        Serial.printf("[HWSM] state=%s gate=%d completed=%d active=%d start=%lu now=%lu\n",
                      outputStateName(state),
                      initPrechargeSucceeded ? 1 : 0,
                      initPrechargeCompleted ? 1 : 0,
                      initPrechargeActive ? 1 : 0,
                      static_cast<unsigned long>(initPrechargeStartMs),
                      static_cast<unsigned long>(now));
        Serial0.printf("[HWSM] state=%s gate=%d completed=%d active=%d start=%lu now=%lu\n",
                       outputStateName(state),
                       initPrechargeSucceeded ? 1 : 0,
                       initPrechargeCompleted ? 1 : 0,
                       initPrechargeActive ? 1 : 0,
                       static_cast<unsigned long>(initPrechargeStartMs),
                       static_cast<unsigned long>(now));
    } else if (pendingState == state) {
        pendingStateSinceMs = now;
    }

    if (state != OutputState::START) {
        startSetpointWaitStartMs = 0;
    }

    applyOutputs(state, prevState, engineThrottlePercent, primaryBrakePercent, inverterReady, inverterFault);
    checkRelayInvariant(state, now);
}

void setStateIdle()
{
    state = initPrechargeSucceeded ? OutputState::IDLE : OutputState::START;
    if (state == OutputState::IDLE) {
        setRelayOutputs(true, false, initPrechargeSucceeded);
    } else {
        setRelayOutputs(false, false);
    }
    writeBrake(0.0f);
    writeDynoThrottle(0.0f);
    setStateLed(state);
}

void setStateMotorDyno()
{
    state = OutputState::MOTOR;
    setRelayOutputs(false, true, initPrechargeSucceeded);
    setStateLed(state);
}

void requestStartReset()
{
    startResetRequestPending = true;
}

bool isMotorState()
{
    return state == OutputState::MOTOR;
}

bool isIdleState()
{
    return state == OutputState::IDLE;
}

void stop()
{
    writeThrottle(0.0f);
    setStateIdle();
}

const char* stateName()
{
    return outputStateName(state);
}

const char* startGateReason()
{
    return startGateReasonText;
}

bool isRbPlusActive()
{
#if METASENSE_RBPLUS_RELAY_ENABLED
    return digitalRead(MetaSense::Globals::kRbPlusRelayPin) == HIGH;
#else
    return false;
#endif
}

void setVcuRelayOverride(bool enabled, bool rbPlus, bool precharge, bool ssr, bool rbMinus)
{
    vcuRelayOverrideEnabled = enabled;
    vcuRbPlusCommand = rbPlus;
    vcuPrechargeCommand = precharge;
    vcuSsrCommand = ssr;
    vcuRbMinusCommand = rbMinus;
}

void applyVcuSimRelayOutputs(bool rbPlusOn, bool prechargeOn, bool ssrOn, bool rbMinusOn)
{
    RelayCommand cmd{};
    cmd.rbMinusOn = rbMinusOn;
    cmd.ssrOn = ssrOn;
    cmd.rbPlusOn = rbPlusOn;
    cmd.prechargeOn = (state == OutputState::START && initPrechargeActive) ? prechargeOn : false;

    // Keep simulation path aligned with production HV relay interlock.
    if (cmd.prechargeOn) {
        cmd.rbPlusOn = false;
    }

    applyRelayCommand(cmd);
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

bool isPrechargeSucceeded()
{
    return initPrechargeSucceeded;
}

bool hasPrestartWarning()
{
    return initPrechargeFailed;
}

} // namespace MetaSense::HardwareOutputStateMachine
