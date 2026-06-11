#include "CANBus.h"
#include "Input.h"

namespace MetaSense::CANBus {

void updateRpm(float rpm)
{
    MetaSense::Input::updateCanRpm(rpm);
}

} // namespace MetaSense::CANBus
