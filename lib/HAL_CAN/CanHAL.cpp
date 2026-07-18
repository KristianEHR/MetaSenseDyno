#include "CanHAL.h"

#include <cstring>
#include "HardwareSerial.h"
#include <driver/gpio.h>

#ifndef METASENSE_LEAF_CAN_TX_ENABLED
// Safety default: allow Leaf CAN receive/monitoring, block all outbound frames.
#define METASENSE_LEAF_CAN_TX_ENABLED 0
#endif

#ifndef METASENSE_LEAF_CAN_TX_SINGLE_SHOT
// Test-oriented default: one transmit attempt per scheduler tick, no auto-retry storm.
#define METASENSE_LEAF_CAN_TX_SINGLE_SHOT 1
#endif

#ifndef METASENSE_LEAF_CAN_LISTEN_ONLY
// Diagnostic default: normal mode unless explicitly compiled for passive bus sniffing.
#define METASENSE_LEAF_CAN_LISTEN_ONLY 0
#endif

#ifndef METASENSE_LEAF_CAN_BITRATE_KBPS
#define METASENSE_LEAF_CAN_BITRATE_KBPS 500
#endif

bool CanHAL::begin(int txPin, int rxPin) {
    if (started_ && txPin_ == txPin && rxPin_ == rxPin) {
        return true;
    }

    if (started_) {
        stop();
    }

    const twai_mode_t mode = METASENSE_LEAF_CAN_LISTEN_ONLY ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)txPin, (gpio_num_t)rxPin, mode);
    
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
    
    // Try a more selective filter: accept only the frames we care about
    // In particular, ensure 0x55B can be received even if previously a TX ID
#if METASENSE_LEAF_CAN_LISTEN_ONLY
    // Listen-only mode: use a filter that explicitly includes 0x55B, 0x1DA, 0x1DC, 0x1D4, etc.
    // This avoids TX object ownership issues by explicitly listing RX IDs
    twai_filter_config_t f_config = {
        .acceptance_code = 0x00000000,  // Match any ID
        .acceptance_mask = 0x00000000,  // No masking = accept all
        .single_filter = false
    };
    Serial.printf("[CAN-INIT] LISTEN_ONLY mode with explicit RX filter\n");
#else
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    Serial.printf("[CAN-INIT] NORMAL mode\n");
#endif
    
    Serial.printf("[CAN-INIT] mode=%d rx_pin=%d tx_pin=%d filter=ACCEPT_ALL\n", 
                  static_cast<int>(mode), rxPin, txPin);

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN-INIT] twai_driver_install FAILED");
        return false;
    }
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
#if METASENSE_LEAF_CAN_TX_SINGLE_SHOT
    msg.flags = TWAI_MSG_FLAG_SS;
#endif
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
    
    // Diagnostic: Check for extended frame bit or other issues
    bool isExtended = (msg.flags & TWAI_MSG_FLAG_EXTD) != 0;
    if (id == 0x55BU || (isExtended && id == 0x55BU)) {
        Serial.printf("[HAL-RX-55B] id=0x%03lX flags=0x%02X dlc=%u extd=%d\n", 
                      static_cast<unsigned long>(id),
                      static_cast<unsigned>(msg.flags),
                      static_cast<unsigned>(msg.data_length_code),
                      isExtended ? 1 : 0);
    }
    
    // Log ALL frame formats every 100 frames to catch frame type distribution
    static uint32_t receiveCount = 0;
    if ((++receiveCount % 100) == 0) {
        Serial.printf("[HAL-RX-STATS] frames_rx=%lu (checking for ext/std distribution)\n", receiveCount);
    }
    
    return true;
}

bool CanHAL::getStatus(twai_status_info_t& outStatus) const {
    if (!started_) {
        return false;
    }
    return twai_get_status_info(&outStatus) == ESP_OK;
}
