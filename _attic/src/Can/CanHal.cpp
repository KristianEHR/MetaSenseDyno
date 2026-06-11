#include <cstring>
#include "CanHal.h"

namespace MetaSense {

bool CanHal_Init() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    if (twai_start() != ESP_OK) return false;
    return true;
}

bool CanHal_Send(uint32_t id, uint8_t dlc, const uint8_t* data) {
    twai_message_t msg = {};
    msg.identifier = id;
    msg.data_length_code = dlc;
    msg.extd = 0;
    msg.rtr = 0;
    memcpy(msg.data, data, dlc);
    return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

bool CanHal_Read(twai_message_t& msg) {
    return twai_receive(&msg, pdMS_TO_TICKS(1)) == ESP_OK;
}

} // namespace MetaSense
