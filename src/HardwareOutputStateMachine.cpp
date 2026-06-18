#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "globals.h"

namespace {

enum class OutputState {
    START,
    IDLE,
    MOTOR,
    DYNO
};

OutputState state = OutputState::START;
OutputState pendingState = OutputState::START;
uint32_t pendingStateSinceMs = 0;
constexpr uint32_t kRelaySwitchDelayMs = 15;
constexpr uint32_t kStateDebounceMs = 50;
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

void setRelayOutputs(bool rbMinusOn, bool sssrOn)
{
    digitalWrite(MetaSense::Globals::kRbMinusFetPin, rbMinusOn ? HIGH : LOW);
    digitalWrite(MetaSense::Globals::kSssrPin, sssrOn ? HIGH : LOW);
}

void writePrimaryBrakeSplit(OutputState hwState, float signedPercent)
{
    signedPercent = constrain(signedPercent, -100.0f, 100.0f);

    switch (hwState) {
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

void applyOutputs(OutputState nextState, OutputState prevState, float engineThrottlePercent, float primaryBrakePercent)
{
    writeThrottleServo(engineThrottlePercent);

    switch (nextState) {
    case OutputState::START:
        setRelayOutputs(false, false);
        if (prevState != OutputState::START) {
            delay(kRelaySwitchDelayMs);
        }
        writePrimaryBrakeSplit(OutputState::START, primaryBrakePercent);
        break;

    case OutputState::IDLE:
        setRelayOutputs(false, false);
        if (prevState != OutputState::IDLE) {
            delay(kRelaySwitchDelayMs);
        }
        setRelayOutputs(true, false);
        writePrimaryBrakeSplit(OutputState::IDLE, primaryBrakePercent);
        break;

    case OutputState::MOTOR:
        setRelayOutputs(false, false);
        if (prevState != OutputState::MOTOR) {
            delay(kRelaySwitchDelayMs);
        }
        setRelayOutputs(false, true);
        writePrimaryBrakeSplit(OutputState::MOTOR, primaryBrakePercent);
        break;

    case OutputState::DYNO:
        setRelayOutputs(false, false);
        if (prevState != OutputState::DYNO) {
            delay(kRelaySwitchDelayMs);
        }
        setRelayOutputs(true, false);
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

    pinMode(MetaSense::Globals::kRbMinusFetPin, OUTPUT);
    pinMode(MetaSense::Globals::kSssrPin, OUTPUT);

    setRelayOutputs(false, false);
    state = OutputState::START;
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

void update(float engineThrottlePercent, float setPoint, float rpm, float primaryBrakePercent)
{
    const OutputState prevState = state;
    OutputState candidateState = state;

    if (setPoint <= 100.0f && rpm <= 100.0f) {
        candidateState = OutputState::START;
    } else if (setPoint >= 200.0f && setPoint <= 2000.0f) {
        candidateState = OutputState::MOTOR;
    } else if (setPoint > 2000.0f && rpm >= setPoint) {
        candidateState = OutputState::DYNO;
    } else if (rpm < 200.0f) {
        candidateState = OutputState::START;
    } else {
        candidateState = OutputState::IDLE;
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

    applyOutputs(state, prevState, engineThrottlePercent, primaryBrakePercent);
}

void setStateStart()
{
    state = OutputState::START;
    setRelayOutputs(false, false);
    writeBrake(0.0f);
    writeDynoThrottle(0.0f);
    setStateLed(state);
}

void setStateIdle()
{
    state = OutputState::IDLE;
    setRelayOutputs(true, false);
    writeBrake(0.0f);
    writeDynoThrottle(0.0f);
    setStateLed(state);
}

void setStateMotorDyno()
{
    state = OutputState::MOTOR;
    setRelayOutputs(false, true);
    setStateLed(state);
}

bool isMotorState()
{
    return state == OutputState::MOTOR;
}

void stop()
{
    writeThrottle(0.0f);
    setStateIdle();
}

} // namespace MetaSense::HardwareOutputStateMachine
