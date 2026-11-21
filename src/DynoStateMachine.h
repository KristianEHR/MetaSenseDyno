#pragma once

#include <cstdint>

namespace MetaSense::DynoStateMachine {

enum class AutoRampProfile : uint8_t {
	Hybrid = 0,
	Linear = 1,
	Exponential = 2,
};

void startRecording();
void stopRecording();
bool isRecording();
bool isAutoRunActive();

void setPanelAuto(bool enabled);
void setAutoMode(bool enabled);
void setManualRpmTarget(float rpm);
void setAutoRampProfile(AutoRampProfile profile);
AutoRampProfile autoRampProfile();
const char* autoRampProfileName();
void update();
void abortAutoRun();
bool isSafetyShutdownActive();
bool isRestartRequired();

void setTorqueFeedForward(float torque);
float torqueFeedForward();
bool isEnergyMeasuring();

} // namespace MetaSense::DynoStateMachine
