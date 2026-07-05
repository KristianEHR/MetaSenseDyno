#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/gpio.h>

#include "globals.h"

namespace {

#ifndef METASENSE_RBPLUS_RELAY_ENABLED
// Default off: release RB+ GPIO from firmware control.
#define METASENSE_RBPLUS_RELAY_ENABLED 0
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
constexpr float kIdleSetpointZeroThresholdRpm = 50.0f;
bool inverterStatusInitialized = false;
bool hvArmedLatched = false;
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

void setupServoPwmChannel(int pin, int channel)
{
    ledcSetup(channel,
              MetaSense::Globals::kServoPwmFrequencyHz,
              MetaSense::Globals::kServoPwmResolutionBits);
    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0);
}

void setupActuatorPwmChannel(int pin, int channel)
{
    ledcSetup(channel,
              MetaSense::Globals::kActuatorPwmFrequencyHz,
              MetaSense::Globals::kActuatorPwmResolutionBits);
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

void writeThrottleServo(float percent)
{
    percent = constrain(percent, 0.0f, 100.0f);

    const float pulseUs = MetaSense::Globals::kServoPulseMinUs +
        (percent / 100.0f) * (MetaSense::Globals::kServoPulseMaxUs - MetaSense::Globals::kServoPulseMinUs);
    const float periodUs = 1000000.0f / static_cast<float>(MetaSense::Globals::kServoPwmFrequencyHz);
    const int maxPwm = pwmMaxValue(MetaSense::Globals::kServoPwmResolutionBits);
    const int pwm = static_cast<int>((pulseUs / periodUs) * static_cast<float>(maxPwm));

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
    cmd.ssrOn = vcuRelayOverrideEnabled ? vcuSsrCommand : sssrOn;

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
        color = strip.Color(255, 200, 0);
        break;
    case OutputState::START:
        color = strip.Color(255, 255, 255);
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

void applyOutputs(OutputState nextState,
                  OutputState prevState,
                  float engineThrottlePercent,
                  float primaryBrakePercent,
                  bool inverterReady)
{
    (void)prevState;
    writeThrottleServo(engineThrottlePercent);
    hvArmedLatched = inverterReady;

    const bool rbPlusHold = hvArmedLatched && nextState != OutputState::INIT;
    const bool prechargeActive = (nextState == OutputState::MOTOR) && !hvArmedLatched;

    switch (nextState) {
    case OutputState::INIT:
        setRelayOutputs(false, false);
        writePrimaryBrakeSplit(OutputState::INIT, 0.0f);
        break;

    case OutputState::START:
        setRelayOutputs(true, false, rbPlusHold);
        writePrimaryBrakeSplit(OutputState::START, primaryBrakePercent);
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

    setupServoPwmChannel(MetaSense::Globals::kThrottlePin,
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
    setStateLed(state);
}

void writeThrottle(float percent)
{
    writeThrottleServo(percent);
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
            bool inverterStatusReady,
            bool inverterReady)
{
    const OutputState prevState = state;
    OutputState candidateState = state;
    const bool idleSetpointZero = setPoint <= kIdleSetpointZeroThresholdRpm;

    if (inverterStatusReady) {
        inverterStatusInitialized = true;
    }

    if (!inverterStatusInitialized) {
        candidateState = OutputState::INIT;
    } else if (!idleSetpointZero && !inverterReady) {
        // Do not enter IDLE until the RPM setpoint POT is returned to zero.
        candidateState = OutputState::INIT;
    } else if (!inverterReady) {
        candidateState = OutputState::IDLE;
    } else if (idleSetpointZero && rpm <= 200.0f) {
        candidateState = OutputState::IDLE;
    } else if (setPoint >= 200.0f && setPoint <= 1500.0f) {
        candidateState = OutputState::MOTOR;
    } else if (setPoint > 1500.0f && rpm >= setPoint) {
        candidateState = OutputState::DYNO;
    } else {
        candidateState = OutputState::MOTOR;
    }

    const uint32_t now = millis();
    if (candidateState != pendingState) {
        pendingState = candidateState;
        pendingStateSinceMs = now;
    }

    if (pendingState != state && (now - pendingStateSinceMs) >= kStateDebounceMs) {
        state = pendingState;
    } else if (pendingState == state) {
        pendingStateSinceMs = now;
    }

    applyOutputs(state, prevState, engineThrottlePercent, primaryBrakePercent, inverterReady);
}

void setStateStart()
{
    // START behavior is functionally equivalent to safe IDLE after CAN status init.
    state = inverterStatusInitialized ? OutputState::IDLE : OutputState::INIT;
    if (state == OutputState::IDLE) {
        setRelayOutputs(true, false, hvArmedLatched);
    } else {
        setRelayOutputs(false, false);
    }
    writeBrake(0.0f);
    writeDynoThrottle(0.0f);
    setStateLed(state);
}

void setStateIdle()
{
    state = inverterStatusInitialized ? OutputState::IDLE : OutputState::INIT;
    if (state == OutputState::IDLE) {
        setRelayOutputs(true, false, hvArmedLatched);
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
    setRelayOutputs(false, true, hvArmedLatched);
    setStateLed(state);
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
    cmd.prechargeOn = prechargeOn;
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

} // namespace MetaSense::HardwareOutputStateMachine
