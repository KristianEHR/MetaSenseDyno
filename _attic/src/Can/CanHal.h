#pragma once
#include <driver/twai.h>

namespace MetaSense {

bool CanHal_Init();
bool CanHal_Send(uint32_t id, uint8_t dlc, const uint8_t* data);
bool CanHal_Read(twai_message_t& msg);

} // namespace MetaSense
