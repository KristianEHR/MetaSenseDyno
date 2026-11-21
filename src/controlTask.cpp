
#include "controlTask.h"

#include "DynoStateMachine.h"
#include "Input.h"
#include "Inverter.h"
#include "PIController.h"
#include "Settings.h"

namespace {

PIController piController;

float clampTorque(float torque, float limitNm)
{
    if (torque < -limitNm) return -limitNm;
    if (torque > limitNm) return limitNm;
    return torque;
}

} // anonymous namespace

namespace MetaSense::ControlTask {

void begin()
{
    MetaSense::DynoStateMachine::setTorqueFeedForward(0.0f);
    configurePI(MetaSense::Settings::kp,
                MetaSense::Settings::ki,
                -MetaSense::Settings::brakeMaxTorqueNm,
                MetaSense::Settings::brakeMaxTorqueNm);
    resetPI();
    MetaSense::Input::begin();
}

void loop()
{
    MetaSense::Input::loop();
}

void configurePI(float kp, float ki, float outMin, float outMax)
{
    piController.setGains(kp, ki);
    piController.setOutputLimits(outMin, outMax);
}

void resetPI()
{
    piController.reset();
}

float computeTorqueCommand(float rpmTarget, float rpmActual, float dtSec)
{
    // Re-applied each cycle so a change to Brake Max Torque takes effect immediately.
    const float limitNm = MetaSense::Settings::brakeMaxTorqueNm;
    piController.setOutputLimits(-limitNm, limitNm);
    const float piTorque = piController.compute(rpmTarget, rpmActual, dtSec);
    const float feedForwardTorque = MetaSense::Inverter::torqueFeedForward();
    return clampTorque(piTorque + feedForwardTorque, limitNm);
}

} // namespace MetaSense::ControlTask
