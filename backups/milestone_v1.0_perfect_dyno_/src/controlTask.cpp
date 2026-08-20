
#include "controlTask.h"

#include "DynoStateMachine.h"
#include "Input.h"
#include "Inverter.h"
#include "PIController.h"
#include "Settings.h"

namespace {

PIController piController;

float clampTorque(float torque)
{
    if (torque < -50.0f) return -50.0f;
    if (torque > 50.0f) return 50.0f;
    return torque;
}

} // anonymous namespace

namespace MetaSense::ControlTask {

void begin()
{
    MetaSense::DynoStateMachine::setTorqueFeedForward(0.0f);
    configurePI(MetaSense::Settings::kp,
                MetaSense::Settings::ki,
                -50.0f,
                50.0f);
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
    const float piTorque = piController.compute(rpmTarget, rpmActual, dtSec);
    const float feedForwardTorque = MetaSense::Inverter::torqueFeedForward();
    return clampTorque(piTorque + feedForwardTorque);
}

} // namespace MetaSense::ControlTask
