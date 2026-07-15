#include "CANBus.h"

#include <string.h>

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
           id == 0x01A || id == 0x01B || id == 0x01C || id == 0x014;
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

    for (uint8_t i = 0; i < s_config.maxFramesPerPoll; ++i) {
        uint32_t id = 0;
        uint8_t len = 0;
        uint8_t data[8] = {0};
        if (!s_canHal.receive(id, data, len)) {
            break;
        }

        twai_message_t msg = {};
        msg.identifier = id;
        msg.data_length_code = len;
        memcpy(msg.data, data, len);

        LeafCan::decodeFrame(msg, s_feedback, nowMs);
        s_stats.lastRxMs = nowMs;
        s_stats.lastRxId = id;
        ++s_stats.rxFrames;
        if (isLeafStatusId(id)) {
            ++s_stats.rxLeafFrames;
        } else {
            ++s_stats.rxUnknownFrames;
            if (id == 0x120U) {
                ++s_stats.rx120Frames;
                s_stats.last120Ms = nowMs;
                s_stats.last120Len = len;
                memset(s_stats.last120Data, 0, sizeof(s_stats.last120Data));
                memcpy(s_stats.last120Data, data, len);
            } else if (id == 0x55AU) {
                ++s_stats.rx55aFrames;
                s_stats.last55aMs = nowMs;
                s_stats.last55aLen = len;
                memset(s_stats.last55aData, 0, sizeof(s_stats.last55aData));
                memcpy(s_stats.last55aData, data, len);
            }
        }
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
