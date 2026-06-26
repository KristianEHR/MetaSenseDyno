#include "CANBus.h"
#include "Input.h"

namespace MetaSense::CANBus {

void updateRpm(float rpm)
{
    MetaSense::Input::updateCanRpm(rpm);
}

void updateLeafFeedback(const LeafInvFeedback& fb)
{
    MetaSense::Input::updateCanRpm(fb.rpm);
    MetaSense::Input::updateCanTorque(fb.torque_nm);
    MetaSense::Input::updateCanTemps(fb.inverter_temp, fb.stator_temp, fb.coolant_temp);
    MetaSense::Input::updateCanStatus(fb.ready, fb.fault, fb.warning, fb.limp);
}

} // namespace MetaSense::CANBus
