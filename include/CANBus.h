#pragma once

#include "LeafCan.h"

namespace MetaSense::CANBus {

void updateRpm(float rpm);
void updateLeafFeedback(const LeafInvFeedback& fb);

} // namespace MetaSense::CANBus
