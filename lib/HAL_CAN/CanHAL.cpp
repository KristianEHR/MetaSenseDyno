#include "CanHAL.h"

#include <cstring>
#include "HardwareSerial.h"
#include <driver/gpio.h>
#include <esp_err.h>
#include "globals.h"

#ifndef METASENSE_LEAF_CAN_TX_ENABLED
// Safety default: allow Leaf CAN receive/monitoring, block all outbound frames.
#define METASENSE_LEAF_CAN_TX_ENABLED 0
#endif

#ifndef METASENSE_LEAF_CAN_RX_ENABLED
// Default to receive enabled so 0x1DA/0x55A monitoring remains active.
#define METASENSE_LEAF_CAN_RX_ENABLED 1
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

#ifndef METASENSE_CAN_HAL_RX_DIAG
#define METASENSE_CAN_HAL_RX_DIAG 0
#endif

#ifndef METASENSE_CAN_RX_ONE_LINE_LOG
#define METASENSE_CAN_RX_ONE_LINE_LOG 0
#endif

namespace {
void logTwaiStatus(const char* tag)
{
    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        Serial.printf("[%s] status_query_failed\n", tag);
        return;
    }

    Serial.printf("[%s] state=%u queued_rx=%lu queued_tx=%lu missed=%lu overrun=%lu arb_lost=%lu bus_err=%lu tx_err=%u rx_err=%u\n",
                  tag,
                  static_cast<unsigned>(status.state),
                  static_cast<unsigned long>(status.msgs_to_rx),
                  static_cast<unsigned long>(status.msgs_to_tx),
                  static_cast<unsigned long>(status.rx_missed_count),
                  static_cast<unsigned long>(status.rx_overrun_count),
                  static_cast<unsigned long>(status.arb_lost_count),
                  static_cast<unsigned long>(status.bus_error_count),
                  static_cast<unsigned>(status.tx_error_counter),
                  static_cast<unsigned>(status.rx_error_counter));
}
}  // namespace

bool CanHAL::begin(int txPin, int rxPin) {
    Serial.printf("[CANHAL-BEGIN] tx=%d rx=%d started=%d\n", txPin, rxPin, started_ ? 1 : 0);
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

    // Keep RX filtering permissive in both modes so protocol bring-up can sniff
    // every frame family (1DA/1DB/1DC/1D4/55B/50B and any variants).
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (METASENSE_LEAF_CAN_LISTEN_ONLY) {
        Serial.printf("[CAN-INIT] LISTEN_ONLY mode\n");
    } else {
        Serial.printf("[CAN-INIT] NORMAL mode\n");
    }

    Serial.printf("[CAN-INIT] mode=%d bitrate=%dk rx_pin=%d tx_pin=%d rx_enabled=%d filter=ACCEPT_ALL\n",
                  static_cast<int>(mode),
                  static_cast<unsigned>(METASENSE_LEAF_CAN_BITRATE_KBPS),
                  rxPin,
                  txPin,
                  static_cast<int>(METASENSE_LEAF_CAN_RX_ENABLED));

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN-INIT] twai_driver_install FAILED");
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }

    logTwaiStatus("CAN-INIT-READY");
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
    const esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        Serial.printf("[CAN-TX] failed id=0x%03lX len=%u err=0x%x\n",
                      static_cast<unsigned long>(id),
                      static_cast<unsigned>(len),
                      static_cast<unsigned>(err));
        return false;
    }
    if (id == 0x1D4U) {
        Serial.printf("[CAN-TX] ok id=0x%03lX len=%u\n",
                      static_cast<unsigned long>(id),
                      static_cast<unsigned>(len));
    }
    return true;
#endif
}

bool CanHAL::receive(uint32_t& id, uint8_t* data, uint8_t& len, bool& isExtended) {
#if !METASENSE_CAN_RX_ONE_LINE_LOG
    Serial.printf("[CANHAL-RECEIVE] started=%d\n", started_ ? 1 : 0);
#endif
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
    const esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(1));
    if (err == ESP_ERR_TIMEOUT) {
#if !METASENSE_CAN_RX_ONE_LINE_LOG
        static uint32_t lastNoFrameLogMs = 0U;
        const uint32_t nowMs = millis();
        if ((nowMs - lastNoFrameLogMs) >= 2000U) {
            lastNoFrameLogMs = nowMs;
            Serial.printf("[CAN-RX] no-frame timeout on twai_receive\n");
            Serial0.printf("[CAN-RX] no-frame timeout on twai_receive\n");
            logTwaiStatus("CAN-RX-TIMEOUT");
        }
#endif
        return false;
    }
    if (err != ESP_OK) {
        static uint32_t lastErrLogMs = 0U;
        const uint32_t nowMs = millis();
        if ((nowMs - lastErrLogMs) >= 2000U) {
            lastErrLogMs = nowMs;
            Serial.printf("[CAN-RX] twai_receive err=0x%X\n", static_cast<unsigned>(err));
            Serial0.printf("[CAN-RX] twai_receive err=0x%X\n", static_cast<unsigned>(err));
        }
        return false;
    }

    id = msg.identifier;
    len = msg.data_length_code;
    if (len > 8U) {
        len = 8U;
    }
    memcpy(data, msg.data, len);
    
    // Surface standard vs extended distribution to upper layers.
    isExtended = (msg.flags & TWAI_MSG_FLAG_EXTD) != 0;
#if !METASENSE_CAN_RX_ONE_LINE_LOG
    Serial.printf("[CAN-RX-FRAME] id=0x%03lX len=%u ext=%d data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                  static_cast<unsigned long>(id),
                  static_cast<unsigned>(len),
                  isExtended ? 1 : 0,
                  static_cast<unsigned>(data[0]),
                  static_cast<unsigned>(data[1]),
                  static_cast<unsigned>(data[2]),
                  static_cast<unsigned>(data[3]),
                  static_cast<unsigned>(data[4]),
                  static_cast<unsigned>(data[5]),
                  static_cast<unsigned>(data[6]),
                  static_cast<unsigned>(data[7]));
#endif
#if METASENSE_CAN_HAL_RX_DIAG
    if (id == 0x55BU || (isExtended && id == 0x55BU)) {
        Serial.printf("[HAL-RX-55B] id=0x%03lX flags=0x%02X dlc=%u extd=%d\n",
                      static_cast<unsigned long>(id),
                      static_cast<unsigned>(msg.flags),
                      static_cast<unsigned>(msg.data_length_code),
                      isExtended ? 1 : 0);
    }

    // Optional frame format heartbeat for deep bus diagnostics only.
    static uint32_t receiveCount = 0;
    if ((++receiveCount % 100) == 0) {
        Serial.printf("[HAL-RX-STATS] frames_rx=%lu\n", static_cast<unsigned long>(receiveCount));
    }
#endif
    
    return true;
}

bool CanHAL::getStatus(twai_status_info_t& outStatus) const {
    if (!started_) {
        return false;
    }
    return twai_get_status_info(&outStatus) == ESP_OK;
}
