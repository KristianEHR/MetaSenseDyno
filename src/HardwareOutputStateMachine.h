#pragma once

namespace MetaSense::HardwareOutputStateMachine {

void begin();

void writeThrottle(float percent);
void writeBrake(float percent);
void writeDynoThrottle(float percent);

void update(float engineThrottlePercent,
            float setPoint,
            float rpm,
            float primaryBrakePercent,
            float inverterHvVoltage,
            bool inverterStatusReady,
            bool inverterReady,
            bool inverterFault);

void setStateIdle();
void setStateMotorDyno();

bool isMotorState();
bool isIdleState();

void stop();
const char* stateName();

bool isRbPlusActive();
void setVcuRelayOverride(bool enabled, bool rbPlus, bool precharge, bool ssr, bool rbMinus);
void applyVcuSimRelayOutputs(bool rbPlusOn, bool prechargeOn, bool ssrOn, bool rbMinusOn);
bool isRbMinusActive();
bool isSsrActive();
bool isPrechargeActive();
bool isPrechargeSucceeded();
bool hasPrestartWarning();

} // namespace MetaSense::HardwareOutputStateMachine