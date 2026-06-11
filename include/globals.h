#pragma once

namespace MetaSense::Globals {

constexpr int kOnboardLedPin = 48;
constexpr int kOnboardLedCount = 1;
constexpr int kI2cSdaPin = 17;
constexpr int kI2cSclPin = 18;

constexpr int kThrottlePin = 45;
constexpr int kBrakePin = 40;
constexpr int kThrottleVcuPin = 41;
constexpr int kRampSwitchPin = 35;
constexpr int kRbPlusInputPin = 36;
constexpr int kRbMinusFetPin = 38;
constexpr int kSssrPin = 39;

constexpr int kThrottlePwmChannel = 0;
constexpr int kBrakePwmChannel = 1;
constexpr int kDynoThrottlePwmChannel = 2;
constexpr int kPwmFrequencyHz = 20000;
constexpr int kPwmResolutionBits = 8;
constexpr unsigned int kHoldingRegisterCount = 15;

} // namespace MetaSense::Globals
