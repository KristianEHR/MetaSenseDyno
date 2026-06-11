#pragma once

namespace MetaSense::DynoStateMachine {

void startRecording();
void stopRecording();
bool isRecording();
bool isAutoRunActive();

void setPanelAuto(bool enabled);
void setAutoMode(bool enabled);
void setManualRpmTarget(float rpm);
void update();
void abortAutoRun();

void setTorqueFeedForward(float torque);
float torqueFeedForward();

} // namespace MetaSense::DynoStateMachine
