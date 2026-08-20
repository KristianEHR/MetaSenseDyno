#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

namespace MetaSense::I2cBus {

bool take(TickType_t timeoutTicks);
void give();

} // namespace MetaSense::I2cBus
