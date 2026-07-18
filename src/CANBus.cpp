#include "CANBus.h"

#include <string.h>
#include "HardwareSerial.h"

#include "CanHAL.h"

namespace MetaSense::CANBus {

namespace {

CanHAL s_canHal;
Config s_config;
bool s_configured = false;
LeafInvFeedback s_feedback{};
Stats s_stats;

bool isLeafStatusId(uint32_t id)
{
    return id == 0x1DA || id == 0x1DB || id == 0x1DC || id == 0x1D4 ||
           id == 0x01A || id == 0x01B || id == 0x01C || id == 0x014 ||
           id == 0x55B || id == 0x05B || id == 0x50B;
}

bool ensureReady(uint32_t nowMs)
{
    if (!s_configured) {
        return false;
    }

    if (s_stats.ready) {
        return true;
    }

    if ((nowMs - s_stats.lastInitAttemptMs) < s_config.initRetryMs) {
        return false;
    }

    s_stats.lastInitAttemptMs = nowMs;
    s_stats.ready = s_canHal.begin(s_config.txPin, s_config.rxPin);
    if (s_stats.ready) {
        LeafCan::reset(s_feedback);
    }
    return s_stats.ready;
}

void handleDriverFault(uint32_t nowMs, bool busOff)
{
    if (!s_stats.ready) {
        return;
    }

    s_canHal.stop();
    s_stats.ready = false;
    s_stats.lastInitAttemptMs = nowMs;
    ++s_stats.recoveries;
    if (busOff) {
        ++s_stats.busOffEvents;
    }
}

} // namespace

void configure(const Config& config)
{
    s_config = config;
    s_configured = true;
}

void reset()
{
    s_canHal.stop();
    LeafCan::reset(s_feedback);
    s_stats = Stats{};
}

void poll(uint32_t nowMs)
{
    if (!ensureReady(nowMs)) {
        return;
    }

    static uint32_t lastDiagMs = 0;
    const uint32_t DIAG_PERIOD_MS = 5000;

    twai_status_info_t twaiStatus = {};
    if (!s_canHal.getStatus(twaiStatus)) {
        ++s_stats.statusQueryFailures;
        handleDriverFault(nowMs, false);
        return;
    }

    s_stats.lastTwaiState = static_cast<uint8_t>(twaiStatus.state);
    s_stats.twaiRxQueued = static_cast<uint32_t>(twaiStatus.msgs_to_rx);
    s_stats.twaiTxQueued = static_cast<uint32_t>(twaiStatus.msgs_to_tx);
    s_stats.twaiRxMissed = static_cast<uint32_t>(twaiStatus.rx_missed_count);
    s_stats.twaiRxOverrun = static_cast<uint32_t>(twaiStatus.rx_overrun_count);
    s_stats.twaiArbLost = static_cast<uint32_t>(twaiStatus.arb_lost_count);
    s_stats.twaiBusError = static_cast<uint32_t>(twaiStatus.bus_error_count);
    s_stats.twaiTxErrorCounter = static_cast<uint32_t>(twaiStatus.tx_error_counter);
    s_stats.twaiRxErrorCounter = static_cast<uint32_t>(twaiStatus.rx_error_counter);
    if (twaiStatus.state == TWAI_STATE_BUS_OFF) {
        handleDriverFault(nowMs, true);
        return;
    }
    if (twaiStatus.state == TWAI_STATE_STOPPED) {
        handleDriverFault(nowMs, false);
        return;
    }
    
    // Periodic diagnostic: show queue status
    if (lastDiagMs == 0U || (nowMs - lastDiagMs) >= DIAG_PERIOD_MS) {
        lastDiagMs = nowMs;
        Serial.printf("[CAN-POLL-DIAG] queued=%lu missed=%lu overrun=%lu arb_lost=%lu bus_err=%lu tx_err=%u rx_err=%u state=%u rx_frames_total=%lu leaf_frames=%lu unk_frames=%lu\n",
                      static_cast<unsigned long>(twaiStatus.msgs_to_rx),
                      static_cast<unsigned long>(twaiStatus.rx_missed_count),
                      static_cast<unsigned long>(twaiStatus.rx_overrun_count),
                      static_cast<unsigned long>(twaiStatus.arb_lost_count),
                      static_cast<unsigned long>(twaiStatus.bus_error_count),
                      static_cast<unsigned>(twaiStatus.tx_error_counter),
                      static_cast<unsigned>(twaiStatus.rx_error_counter),
                      static_cast<unsigned>(twaiStatus.state),
                      static_cast<unsigned long>(s_stats.rxFrames),
                      static_cast<unsigned long>(s_stats.rxLeafFrames),
                      static_cast<unsigned long>(s_stats.rxUnknownFrames));
    }

    uint32_t framesThisPoll = 0;
    uint32_t extFramesThisPoll = 0;

    for (uint8_t i = 0; i < s_config.maxFramesPerPoll; ++i) {
        uint32_t id = 0;
        uint8_t len = 0;
        uint8_t data[8] = {0};
        if (!s_canHal.receive(id, data, len)) {
            break;
        }
        
        ++framesThisPoll;

        twai_message_t msg = {};
        msg.identifier = id;
        msg.data_length_code = len;
        memcpy(msg.data, data, len);

        LeafCan::decodeFrame(msg, s_feedback, nowMs);
        s_stats.lastRxMs = nowMs;
        s_stats.lastRxId = id;
        ++s_stats.rxFrames;
        
        // Track both IDs explicitly to disambiguate 0x55B vs 0x05B bus variants.
        if (id == 0x55BU || id == 0x05BU || id == 0x50BU) {
            if (id == 0x55BU) {
                ++s_stats.rx55bFrames;
            } else if (id == 0x05BU) {
                ++s_stats.rx05bFrames;
            } else {
                ++s_stats.rx50bFrames;
            }
            s_stats.last55bMs = nowMs;
            s_stats.last55bLen = len;
            Serial.printf("[CAN-55B-RX] id=0x%03lX len=%u data=%02X %02X %02X %02X\n",
                          static_cast<unsigned long>(id),
                          static_cast<unsigned>(len),
                          static_cast<unsigned>(data[0]),
                          static_cast<unsigned>(data[1]),
                          static_cast<unsigned>(data[2]),
                          static_cast<unsigned>(data[3]));
        }
        
        // Log ALL arriving frames for debugging
        Serial.printf("[CAN-FRAME-RX] id=0x%03lX len=%u data=%02X %02X %02X %02X\n",
                      static_cast<unsigned long>(id),
                      static_cast<unsigned>(len),
                      static_cast<unsigned>(data[0]),
                      static_cast<unsigned>(data[1]),
                      static_cast<unsigned>(data[2]),
                      static_cast<unsigned>(data[3]));
        
        if (isLeafStatusId(id)) {
            ++s_stats.rxLeafFrames;
        } else {
            ++s_stats.rxUnknownFrames;
            s_stats.lastUnknownMs = nowMs;
            s_stats.lastUnknownId = id;
            s_stats.lastUnknownLen = len;
            memset(s_stats.lastUnknownData, 0, sizeof(s_stats.lastUnknownData));
            memcpy(s_stats.lastUnknownData, data, len);
            if (id == 0x55AU) {
                ++s_stats.rx55aFrames;
                s_stats.last55aMs = nowMs;
                s_stats.last55aLen = len;
                memset(s_stats.last55aData, 0, sizeof(s_stats.last55aData));
                memcpy(s_stats.last55aData, data, len);
            }
        }
    }
    
    if (lastDiagMs == 0U || (nowMs - lastDiagMs) >= DIAG_PERIOD_MS) {
        Serial.printf("[CAN-POLL-FRAME-COUNT] this_poll=%lu ext_frames=%lu\n", static_cast<unsigned long>(framesThisPoll), static_cast<unsigned long>(extFramesThisPoll));
    }
}

bool send(uint32_t id, const uint8_t* data, uint8_t len)
{
    if (!s_stats.ready) {
        if (!s_stats.txWhileNotReadyLatched) {
            ++s_stats.txWhileNotReady;
            s_stats.txWhileNotReadyLatched = true;
        }
        return false;
    }

    s_stats.txWhileNotReadyLatched = false;

    const bool sent = s_canHal.send(id, data, len);
    if (sent) {
        ++s_stats.txFrames;
        s_stats.txFailureLatched = false;
    } else {
        if (!s_stats.txFailureLatched) {
            ++s_stats.txFailures;
            s_stats.txFailureLatched = true;
        }
    }
    return sent;
}

bool isReady()
{
    return s_stats.ready;
}

const LeafInvFeedback& feedback()
{
    return s_feedback;
}

const Stats& stats()
{
    return s_stats;
}

} // namespace MetaSense::CANBus
