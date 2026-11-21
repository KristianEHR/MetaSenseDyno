#include "CanHAL.h"

#include <cstring>
#include "HardwareSerial.h"
#include <driver/gpio.h>
#include <esp_err.h>
#include "globals.h"
#include "../include/CanConfig.h"  // For METASENSE_LEAF_CAN_RX_QUEUE_LEN, TX_QUEUE_LEN

#ifndef METASENSE_LEAF_CAN_TX_ENABLED
#define METASENSE_LEAF_CAN_TX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_CAN_RX_ENABLED
#define METASENSE_LEAF_CAN_RX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_CAN_LISTEN_ONLY
#define METASENSE_LEAF_CAN_LISTEN_ONLY 0
#endif

#ifndef METASENSE_LEAF_CAN_BITRATE_KBPS
#define METASENSE_LEAF_CAN_BITRATE_KBPS 500
#endif

// Standard TWAI setup: driver install + start, normal (non-listen-only) mode
// unless explicitly configured otherwise. No single-shot transmit flag -- the
// controller handles bus-idle sensing, bitwise arbitration and ACK/NACK with
// automatic retransmit exactly as any standard CAN node.
bool CanHAL::begin(int txPin, int rxPin) {
    if (started_ && txPin_ == txPin && rxPin_ == rxPin) {
        return true;
    }

    if (started_) {
        stop();
    }

    const twai_mode_t mode = METASENSE_LEAF_CAN_LISTEN_ONLY ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)txPin, (gpio_num_t)rxPin, mode);
    g_config.rx_queue_len = METASENSE_LEAF_CAN_RX_QUEUE_LEN;
    g_config.tx_queue_len = METASENSE_LEAF_CAN_TX_QUEUE_LEN;

    twai_timing_config_t t_config;
#if METASENSE_LEAF_CAN_BITRATE_KBPS == 1000
    t_config = TWAI_TIMING_CONFIG_1MBITS();
#elif METASENSE_LEAF_CAN_BITRATE_KBPS == 500
    t_config = TWAI_TIMING_CONFIG_500KBITS();
#elif METASENSE_LEAF_CAN_BITRATE_KBPS == 250
    t_config = TWAI_TIMING_CONFIG_250KBITS();
#elif METASENSE_LEAF_CAN_BITRATE_KBPS == 125
    t_config = TWAI_TIMING_CONFIG_125KBITS();
#else
#error Unsupported METASENSE_LEAF_CAN_BITRATE_KBPS. Use one of: 125, 250, 500, 1000.
#endif

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    Serial.printf("[CAN-INIT] mode=%s bitrate=%dk tx_pin=%d rx_pin=%d rx_queue=%lu tx_queue=%lu\n",
                  METASENSE_LEAF_CAN_LISTEN_ONLY ? "LISTEN_ONLY" : "NORMAL",
                  static_cast<unsigned>(METASENSE_LEAF_CAN_BITRATE_KBPS),
                  txPin,
                  rxPin,
                  static_cast<unsigned long>(g_config.rx_queue_len),
                  static_cast<unsigned long>(g_config.tx_queue_len));

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN-INIT] twai_driver_install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN-INIT] twai_start failed");
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

    // Standard transmit: the controller queues the frame, waits for bus-idle,
    // arbitrates, and (on lost arbitration or missing ACK) retries
    // automatically. ESP_OK here means the frame was accepted into the
    // driver's queue, not necessarily that it has been ACKed on the wire yet.
    return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
#endif
}

bool CanHAL::receive(uint32_t& id, uint8_t* data, uint8_t& len, bool& isExtended) {
    if (!started_) {
        return false;
    }

#if !METASENSE_LEAF_CAN_RX_ENABLED
    (void)id;
    (void)data;
    (void)len;
    (void)isExtended;
    return false;
#endif

    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(1)) != ESP_OK) {
        return false;
    }

    id = msg.identifier;
    len = msg.data_length_code;
    if (len > 8U) {
        len = 8U;
    }
    memcpy(data, msg.data, len);
    isExtended = (msg.flags & TWAI_MSG_FLAG_EXTD) != 0;
    return true;
}

bool CanHAL::getStatus(twai_status_info_t& outStatus) const {
    if (!started_) {
        return false;
    }
    return twai_get_status_info(&outStatus) == ESP_OK;
}
