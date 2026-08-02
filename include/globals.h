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

#ifndef METASENSE_GPIO_START_REQUEST_PIN
#define METASENSE_GPIO_START_REQUEST_PIN 37
#endif

constexpr int kOnboardLedPin = 48;
constexpr int kOnboardLedCount = 1;
constexpr int kI2cSdaPin = 17;
constexpr int kI2cSclPin = 18;

constexpr int kThrottlePin = METASENSE_GPIO_THROTTLE_PIN;
constexpr int kBrakePin = 40;
constexpr int kThrottleVcuPin = 41;
constexpr int kRampSwitchPin = 35;
constexpr int kStartRequestPin = METASENSE_GPIO_START_REQUEST_PIN;
constexpr int kRbPlusInputPin = 36;
constexpr int kRbPlusRelayPin = METASENSE_GPIO_RBPLUS_PIN;
constexpr int kPrechargeRelayPin = METASENSE_GPIO_PRECHARGE_PIN;
constexpr int kRbMinusFetPin = METASENSE_GPIO_RBMINUS_PIN;
constexpr int kSssrPin = METASENSE_GPIO_SSR_PIN;

constexpr int kThrottlePwmChannel = 0;
constexpr int kBrakePwmChannel = 2;
constexpr int kDynoThrottlePwmChannel = 4;

#ifndef METASENSE_THROTTLE_PWM_FREQUENCY_HZ
#define METASENSE_THROTTLE_PWM_FREQUENCY_HZ 250
#endif
#ifndef METASENSE_THROTTLE_PWM_RESOLUTION_BITS
#define METASENSE_THROTTLE_PWM_RESOLUTION_BITS 12
#endif
#ifndef METASENSE_THROTTLE_PWM_MIN_PERCENT
#define METASENSE_THROTTLE_PWM_MIN_PERCENT 0.0f
#endif
#ifndef METASENSE_THROTTLE_PWM_MAX_PERCENT
#define METASENSE_THROTTLE_PWM_MAX_PERCENT 100.0f
#endif

constexpr int kThrottlePwmFrequencyHz = METASENSE_THROTTLE_PWM_FREQUENCY_HZ;
constexpr int kThrottlePwmResolutionBits = METASENSE_THROTTLE_PWM_RESOLUTION_BITS;
constexpr float kThrottlePwmMinPercent = METASENSE_THROTTLE_PWM_MIN_PERCENT;
constexpr float kThrottlePwmMaxPercent = METASENSE_THROTTLE_PWM_MAX_PERCENT;

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
static_assert(kStartRequestPin != kRampSwitchPin, "Start-request pin overlaps ramp switch pin");
static_assert(kStartRequestPin != kRbPlusRelayPin, "Start-request pin overlaps RB+ relay pin");
static_assert(kStartRequestPin != kPrechargeRelayPin, "Start-request pin overlaps precharge relay pin");
static_assert(kStartRequestPin != kRbMinusFetPin, "Start-request pin overlaps RB- relay pin");
static_assert(kStartRequestPin != kSssrPin, "Start-request pin overlaps SSR relay pin");

} // namespace MetaSense::Globals
