#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncWebServer.h>
#include <driver/gpio.h>

#include "HardwareOutputStateMachine.h"
#include "HardwareOutputConfig.h"
#include "globals.h"
#include "WebSocketServer.h"

namespace {

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
// Keep INIT in place until the full startup stack is truly ready. With the
// inverter silent, this prevents an early transition to START.
constexpr uint32_t kInitPrereqStableMs = 0;
constexpr uint32_t kInitPrechargeMinMs = 2500;
constexpr uint32_t kStartSetpointWaitMs = 15000;
constexpr float kHvRelayHighV = 320.0f;
constexpr float kHvRelayLowV = 300.0f;
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
const char* startGateReasonText = "open";
bool motorStartRequestPending = false;
float motorStartRequestedRpm = 0.0f;
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
    case OutputState::INIT:
        expected.rbMinusOn = false;
        expected.ssrOn = false;
        expected.rbPlusOn = false;
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

bool relayInterlockPasses(const RelayCommand& cmd)
{
    return !(cmd.ssrOn && cmd.rbMinusOn);
}

bool relayInterlockViolationFor(OutputState hwState, bool rssOn, bool rbMinusOn)
{
    (void)hwState;
    return rssOn && rbMinusOn;
}

void logRelayInterlockViolation(OutputState hwState,
                                const RelayCommand& cmd,
                                uint32_t now)
{
    static uint32_t lastLogMs = 0;
    const bool logNow = (now - lastLogMs) >= kRelayInvariantLogPeriodMs;
    if (!logNow) {
        return;
    }

    Serial.printf("[HWSM][CHECK] invalid combo state=%s RSS=%d RB-=%d\n",
                  outputStateName(hwState),
                  cmd.ssrOn ? 1 : 0,
                  cmd.rbMinusOn ? 1 : 0);
    Serial0.printf("[HWSM][CHECK] invalid combo state=%s RSS=%d RB-=%d\n",
                   outputStateName(hwState),
                   cmd.ssrOn ? 1 : 0,
                   cmd.rbMinusOn ? 1 : 0);
    lastLogMs = now;
}

void logAllInvalidRelayInterlockCombos()
{
    static bool logged = false;
    if (logged) {
        return;
    }

    const OutputState states[] = {
        OutputState::INIT,
        OutputState::START,
        OutputState::IDLE,
        OutputState::MOTOR,
        OutputState::DYNO
    };

    for (const OutputState hwState : states) {
        for (int rssOn = 0; rssOn <= 1; ++rssOn) {
            for (int rbMinusOn = 0; rbMinusOn <= 1; ++rbMinusOn) {
                const RelayCommand cmd{};
                (void)cmd;
                if ((rssOn != 0) && (rbMinusOn != 0)) {
                    Serial.printf("[HWSM][CHECK] invalid combo state=%s RSS=%d RB-=%d\n",
                                  outputStateName(hwState),
                                  rssOn,
                                  rbMinusOn);
                    Serial0.printf("[HWSM][CHECK] invalid combo state=%s RSS=%d RB-=%d\n",
                                   outputStateName(hwState),
                                   rssOn,
                                   rbMinusOn);
                }
            }
        }
    }

    logged = true;
}

void applyRelayCommand(const RelayCommand& target)
{
    RelayCommand normalized = target;
    if (!relayInterlockPasses(normalized)) {
        logRelayInterlockViolation(state, normalized, millis());
        normalized.ssrOn = false;
        normalized.rbMinusOn = false;
    }

    if (relayCommandEquals(activeRelayCommand, normalized)) {
        return;
    }

    const bool rbMinusGoingLow = activeRelayCommand.rbMinusOn && !normalized.rbMinusOn;
    const bool rbMinusGoingHigh = !activeRelayCommand.rbMinusOn && normalized.rbMinusOn;
    const bool ssrGoingLow = activeRelayCommand.ssrOn && !normalized.ssrOn;
    const bool ssrGoingHigh = !activeRelayCommand.ssrOn && normalized.ssrOn;
    const bool rbPlusGoingLow = activeRelayCommand.rbPlusOn && !normalized.rbPlusOn;
    const bool rbPlusGoingHigh = !activeRelayCommand.rbPlusOn && normalized.rbPlusOn;
    const bool prechargeGoingLow = activeRelayCommand.prechargeOn && !normalized.prechargeOn;
    const bool prechargeGoingHigh = !activeRelayCommand.prechargeOn && normalized.prechargeOn;

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

    writeRelayPinsImmediate(normalized);
    activeRelayCommand = normalized;
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

        initPrechargeSucceeded = true;
        initPrechargeFailed = false;

        if (!initPrechargeSucceeded && !prestartWarnPrinted) {
            Serial.printf("[HWSM][PRESTART_WARN] START precharge completed without a specific HV gate: hv=%.1fV\n",
                          inverterHvVoltage);
            Serial0.printf("[HWSM][PRESTART_WARN] START precharge completed without a specific HV gate: hv=%.1fV\n",
                           inverterHvVoltage);
            prestartWarnPrinted = true;
        }
    }
}

void applyOutputs(OutputState nextState,
                  OutputState prevState,
                  float engineThrottlePercent,
                  float primaryBrakePercent,
                  bool inverterReady,
                  bool inverterFault,
                  float inverterHvVoltage)
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

    const bool hvHigh = inverterHvVoltage >= kHvRelayHighV;
    const bool hvLow = inverterHvVoltage < kHvRelayLowV;
    const bool prechargeActive = initPrechargeActive;
    const bool rbPlusHold = initPrechargeSucceeded;
    // Keep the HV path energized in the run states so the inverter rail stays
    // above 300 V even when no HV battery is present on the dyno bench.
    const bool keepHvAlive = (nextState != OutputState::INIT) && !inverterFault;
    const bool hvStayAliveEnabled = keepHvAlive || hvHigh || hvLow;

    switch (nextState) {
    case OutputState::INIT:
        // INIT contract: firmware/hardware bring-up complete, keep relays OFF.
        setRelayOutputs(false, false, false, false);
        writePrimaryBrakeSplit(OutputState::INIT, 0.0f);
        break;

    case OutputState::START:
        if (hvStayAliveEnabled) {
            // The bench stay-alive path keeps HV present with SSR asserted and
            // RB- released, which avoids the interlock conflict while still
            // maintaining the HV rail above 300 V.
            setRelayOutputs(false, true, true, true);
        } else {
            setRelayOutputs(false, prechargeActive, false, prechargeActive);
        }
        writePrimaryBrakeSplit(OutputState::START, 0.0f);
        break;

    case OutputState::IDLE:
        if (hvStayAliveEnabled) {
            setRelayOutputs(false, true, true, true);
        } else {
            setRelayOutputs(true, false, rbPlusHold, false);
        }
        writePrimaryBrakeSplit(OutputState::IDLE, primaryBrakePercent);
        break;

    case OutputState::MOTOR:
        if (hvStayAliveEnabled) {
            setRelayOutputs(false, true, true, true);
        } else {
            setRelayOutputs(false, true, rbPlusHold, prechargeActive);
        }
        writePrimaryBrakeSplit(OutputState::MOTOR, primaryBrakePercent);
        break;

    case OutputState::DYNO:
        if (hvStayAliveEnabled) {
            setRelayOutputs(false, true, true, true);
        } else {
            setRelayOutputs(true, false, rbPlusHold, false);
        }
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
    const bool rbPlusValid = configureOutputPin(MetaSense::Globals::kRbPlusRelayPin);
    const bool prechargeValid = configureOutputPin(MetaSense::Globals::kPrechargeRelayPin);
    (void)rbMinusValid;
    (void)ssrValid;
    (void)rbPlusValid;
    (void)prechargeValid;

    activeRelayCommand = RelayCommand{};
    logAllInvalidRelayInterlockCombos();
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
            bool telemetryConnected,
            bool canTelemetryReady,
            bool sensorsReady)
{
    const OutputState prevState = state;
    OutputState candidateState = state;
    const bool idleSetpointZero = setPoint <= kIdleSetpointZeroThresholdRpm;
    const bool startExitSetpointZero = (fabsf(setPoint) <= 0.001f);
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

    if (inverterStatusReady || canTelemetryReady) {
        inverterStatusInitialized = true;
    }

    const uint32_t now = millis();
    bool motorStartByRequest = false;
    if (motorStartRequestPending) {
        motorStartRequestPending = false;
        motorStartByRequest = true;

        initPrechargeActive = false;
        initPrechargeCompleted = false;
        initPrechargeSucceeded = false;
        initPrechargeFailed = false;
        prestartWarnPrinted = false;
        initPrechargeStartMs = 0;
        startSetpointWaitStartMs = 0;
    }

    // Hardware readiness determined by actual inverter status bit from 0x1DA frame.
    // inverterStatusReady reflects inv_status_bit = 1 (inverter is ready),
    // which is passed from Input.cpp and provides the definitive INIT criterion.
    const bool hardwareReady = inverterStatusReady;
    // Telemetry is useful for the UI, but it should not keep the controller
    // stuck in INIT/white LED during boot when the core hardware and sensors
    // are already healthy. Poor or intermittent telemetry should not block the
    // state machine from entering IDLE or MOTOR.
    const bool coreBootReady = sensorsReady && hardwareReady;
    const bool initPrereqNow = coreBootReady;
    if (initPrereqNow) {
        if (initReadySinceMs == 0U) {
            initReadySinceMs = now;
        }
    } else {
        initReadySinceMs = 0U;
    }
    const bool initPrereqStable = (initReadySinceMs != 0U) &&
                                  ((kInitPrereqStableMs == 0U) ||
                                   ((now - initReadySinceMs) >= kInitPrereqStableMs));

    const bool allowStartPrecharge = (state == OutputState::START);

    if (allowStartPrecharge) {
        updateStartPrechargeSequence(setPoint, inverterHvVoltage, inverterFault, now);
    } else {
        initPrechargeActive = false;
    }

    if (motorStartByRequest) {
        candidateState = OutputState::MOTOR;
    } else if (state == OutputState::INIT) {
        // INIT is the full firmware/hardware bring-up phase. Once the startup
        // prerequisites are stable, move straight to the run state implied by
        // the current setpoint so the LED and state stay consistent.
        candidateState = initPrereqStable
            ? (idleSetpointZero ? OutputState::IDLE : OutputState::MOTOR)
            : OutputState::INIT;
    } else if (state == OutputState::START) {
        // Keep the startup gate short-lived. A zero setpoint returns to IDLE,
        // while any positive setpoint should immediately progress to MOTOR.
        startSetpointWaitStartMs = 0;
        candidateState = motorSetpointCondition
            ? OutputState::MOTOR
            : OutputState::IDLE;
    } else if (inverterFault) {
        candidateState = OutputState::START;
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
    if (motorStartByRequest) {
        startGateReasonText = "motor_start_button";
    } else if (state == OutputState::INIT) {
        if (initPrereqStable) {
            startGateReasonText = "init_to_start";
        } else if (!hardwareReady) {
            startGateReasonText = "init_wait_hw";
        } else if (!sensorsReady) {
            startGateReasonText = "init_wait_sensors";
        } else {
            startGateReasonText = "init_wait_stable";
        }
    } else if (inverterFault && state != OutputState::START) {
        startGateReasonText = "fault";
    } else if (state == OutputState::START && !initPrechargeSucceeded) {
        if (setPoint > kIdleSetpointZeroThresholdRpm) {
            startGateReasonText = "wait_setpoint_zero";
        } else if (initPrechargeCompleted) {
            startGateReasonText = "precharge_wait_hv";
        } else if (initPrechargeActive) {
            startGateReasonText = "precharge_active";
        } else {
            startGateReasonText = "precharge_pending";
        }
    } else if (state == OutputState::START) {
        startGateReasonText = startExitSetpointZero ? "start_exit_idle" : "wait_setpoint_zero";
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

    applyOutputs(state, prevState, engineThrottlePercent, primaryBrakePercent, inverterReady, inverterFault, inverterHvVoltage);
    checkRelayInvariant(state, now);
}

void setStateIdle()
{
    state = OutputState::IDLE;
    setRelayOutputs(false, false, true, true);
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
    return digitalRead(MetaSense::Globals::kRbPlusRelayPin) == HIGH;
}

bool isRbPlusCommandedActive()
{
    return activeRelayCommand.rbPlusOn;
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

bool isRbMinusCommandedActive()
{
    return activeRelayCommand.rbMinusOn;
}

bool isSsrActive()
{
    return digitalRead(MetaSense::Globals::kSssrPin) == HIGH;
}

bool isSsrCommandedActive()
{
    return activeRelayCommand.ssrOn;
}

bool isPrechargeActive()
{
    return digitalRead(MetaSense::Globals::kPrechargeRelayPin) == HIGH;
}

bool isPrechargeCommandedActive()
{
    return activeRelayCommand.prechargeOn;
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
