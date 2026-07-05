#pragma once

namespace MetaSense::Globals {

extern const bool kVcuSwitch;

#ifndef METASENSE_GPIO_RBPLUS_PIN
#define METASENSE_GPIO_RBPLUS_PIN 36
#endif

#ifndef METASENSE_GPIO_PRECHARGE_PIN
#define METASENSE_GPIO_PRECHARGE_PIN 38
#endif

#ifndef METASENSE_GPIO_RBMINUS_PIN
#define METASENSE_GPIO_RBMINUS_PIN 45
#endif

#ifndef METASENSE_GPIO_SSR_PIN
#define METASENSE_GPIO_SSR_PIN 39
#endif

#ifndef METASENSE_GPIO_THROTTLE_PIN
#define METASENSE_GPIO_THROTTLE_PIN 47
#endif

constexpr int kOnboardLedPin = 48;
constexpr int kOnboardLedCount = 1;
constexpr int kI2cSdaPin = 17;
constexpr int kI2cSclPin = 18;

constexpr int kThrottlePin = METASENSE_GPIO_THROTTLE_PIN;
constexpr int kBrakePin = 40;
constexpr int kThrottleVcuPin = 41;
constexpr int kRampSwitchPin = 35;
constexpr int kRbPlusInputPin = 36;
constexpr int kRbPlusRelayPin = METASENSE_GPIO_RBPLUS_PIN;
constexpr int kPrechargeRelayPin = METASENSE_GPIO_PRECHARGE_PIN;
constexpr int kRbMinusFetPin = METASENSE_GPIO_RBMINUS_PIN;
constexpr int kSssrPin = METASENSE_GPIO_SSR_PIN;

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

static_assert(kThrottlePin != kRbMinusFetPin, "Throttle pin overlaps RB- relay pin");
static_assert(kThrottlePin != kRbPlusRelayPin, "Throttle pin overlaps RB+ relay pin");
static_assert(kThrottlePin != kPrechargeRelayPin, "Throttle pin overlaps precharge relay pin");
static_assert(kThrottlePin != kSssrPin, "Throttle pin overlaps SSR relay pin");

} // namespace MetaSense::Globals
