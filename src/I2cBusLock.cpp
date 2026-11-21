#include "I2cBusLock.h"

#include <freertos/semphr.h>

namespace MetaSense::I2cBus {

namespace {

SemaphoreHandle_t i2cMutex = nullptr;

SemaphoreHandle_t ensureMutex()
{
    if (i2cMutex == nullptr) {
        i2cMutex = xSemaphoreCreateMutex();
    }
    return i2cMutex;
}

} // namespace

bool take(TickType_t timeoutTicks)
{
    SemaphoreHandle_t mutex = ensureMutex();
    if (mutex == nullptr) {
        return false;
    }
    return xSemaphoreTake(mutex, timeoutTicks) == pdTRUE;
}

void give()
{
    if (i2cMutex != nullptr) {
        xSemaphoreGive(i2cMutex);
    }
}

} // namespace MetaSense::I2cBus
