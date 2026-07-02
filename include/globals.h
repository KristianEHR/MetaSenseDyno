#pragma once

namespace MetaSense::Globals {

extern const bool kVcuSwitch;

#ifndef METASENSE_GPIO_RPLUS_PIN
#define METASENSE_GPIO_RPLUS_PIN 37
#endif

#ifndef METASENSE_GPIO_PRECHARGE_PIN
#define METASENSE_GPIO_PRECHARGE_PIN 42
#endif

constexpr int kOnboardLedPin = 48;
constexpr int kOnboardLedCount = 1;
constexpr int kI2cSdaPin = 17;
constexpr int kI2cSclPin = 18;

constexpr int kThrottlePin = 45;
constexpr int kBrakePin = 40;
constexpr int kThrottleVcuPin = 41;
constexpr int kRampSwitchPin = 35;
constexpr int kRbPlusInputPin = 36;
constexpr int kRPlusRelayPin = METASENSE_GPIO_RPLUS_PIN;
constexpr int kPrechargeRelayPin = METASENSE_GPIO_PRECHARGE_PIN;
constexpr int kRbMinusFetPin = 38;
constexpr int kSssrPin = 39;

constexpr int kThrottlePwmChannel = 0;
constexpr int kBrakePwmChannel = 2;
constexpr int kDynoThrottlePwmChannel = 4;

constexpr int kServoPwmFrequencyHz = 50;
constexpr int kServoPwmResolutionBits = 14;
constexpr int kServoPulseMinUs = 1000;
constexpr int kServoPulseMaxUs = 2000;

constexpr int kActuatorPwmFrequencyHz = 5000;
constexpr int kActuatorPwmResolutionBits = 10;
constexpr unsigned int kHoldingRegisterCount = 20;

} // namespace MetaSense::Globals
