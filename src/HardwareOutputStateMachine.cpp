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

void writeChannel(int channel, float percent)
{
    percent = constrain(percent, 0.0f, 100.0f);
    int pwm = (int)(percent * 255.0f / 100.0f);
    ledcWrite(channel, pwm);
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
        writeChannel(MetaSense::Globals::kBrakePwmChannel,
                     signedPercent > 0.0f ? signedPercent : 0.0f);
        writeChannel(MetaSense::Globals::kDynoThrottlePwmChannel, 0.0f);
        break;

    case OutputState::DYNO:
        writeChannel(MetaSense::Globals::kBrakePwmChannel, 0.0f);
        writeChannel(MetaSense::Globals::kDynoThrottlePwmChannel,
                     signedPercent < 0.0f ? -signedPercent : 0.0f);
        break;

    case OutputState::IDLE:
        writeChannel(MetaSense::Globals::kBrakePwmChannel, 0.0f);
        writeChannel(MetaSense::Globals::kDynoThrottlePwmChannel, 0.0f);
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
    writeChannel(MetaSense::Globals::kThrottlePwmChannel, engineThrottlePercent);

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

    ledcSetup(MetaSense::Globals::kThrottlePwmChannel,
              MetaSense::Globals::kPwmFrequencyHz,
              MetaSense::Globals::kPwmResolutionBits);
    ledcAttachPin(MetaSense::Globals::kThrottlePin,
                  MetaSense::Globals::kThrottlePwmChannel);

    ledcSetup(MetaSense::Globals::kBrakePwmChannel,
              MetaSense::Globals::kPwmFrequencyHz,
              MetaSense::Globals::kPwmResolutionBits);
    ledcAttachPin(MetaSense::Globals::kBrakePin,
                  MetaSense::Globals::kBrakePwmChannel);

    ledcSetup(MetaSense::Globals::kDynoThrottlePwmChannel,
              MetaSense::Globals::kPwmFrequencyHz,
              MetaSense::Globals::kPwmResolutionBits);
    ledcAttachPin(MetaSense::Globals::kThrottleVcuPin,
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
    writeChannel(MetaSense::Globals::kThrottlePwmChannel, percent);
}

void writeBrake(float percent)
{
    writeChannel(MetaSense::Globals::kBrakePwmChannel, percent);
}

void writeDynoThrottle(float percent)
{
    writeChannel(MetaSense::Globals::kDynoThrottlePwmChannel, percent);
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

void stop()
{
    writeThrottle(0.0f);
    setStateIdle();
}

} // namespace MetaSense::HardwareOutputStateMachine
