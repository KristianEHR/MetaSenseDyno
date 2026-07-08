#include "CanHAL.h"

#include <cstring>

#ifndef METASENSE_LEAF_CAN_TX_ENABLED
// Safety default: allow Leaf CAN receive/monitoring, block all outbound frames.
#define METASENSE_LEAF_CAN_TX_ENABLED 0
#endif

bool CanHAL::begin(int txPin, int rxPin) {
    if (started_ && txPin_ == txPin && rxPin_ == rxPin) {
        return true;
    }

    if (started_) {
        stop();
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)txPin, (gpio_num_t)rxPin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }

    started_ = true;
    txPin_ = txPin;
    rxPin_ = rxPin;
    return true;
}

void CanHAL::stop() {
    if (!started_) {
        return;
    }

    twai_stop();
    twai_driver_uninstall();
    started_ = false;
    txPin_ = -1;
    rxPin_ = -1;
}

bool CanHAL::send(uint32_t id, const uint8_t* data, uint8_t len) {
#if !METASENSE_LEAF_CAN_TX_ENABLED
    (void)id;
    (void)data;
    (void)len;
    return false;
#else
    if (!started_) {
        return false;
    }

    if (len > 8U) {
        len = 8U;
    }

    twai_message_t msg = {};
    msg.identifier = id;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);
    return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
#endif
}

bool CanHAL::receive(uint32_t& id, uint8_t* data, uint8_t& len) {
    if (!started_) {
        return false;
    }

    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) return false;

    id = msg.identifier;
    len = msg.data_length_code;
    if (len > 8U) {
        len = 8U;
    }
    memcpy(data, msg.data, len);
    return true;
}

bool CanHAL::getStatus(twai_status_info_t& outStatus) const {
    if (!started_) {
        return false;
    }
    return twai_get_status_info(&outStatus) == ESP_OK;
}
