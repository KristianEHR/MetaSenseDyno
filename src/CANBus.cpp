#include "CANBus.h"

#include <string.h>
#include "HardwareSerial.h"
#include "LeafCrc.h"
#include "CanHAL.h"

namespace MetaSense::CANBus {

namespace {

uint8_t compute1daWireCrcFixed(const uint8_t* payload8)
{
    if (payload8 == nullptr) {
        return 0U;
    }
    return MetaSense::LeafCRC::computeExact1daWireCrc(0xDAU, payload8);
}

bool is1daWireCrcKnownGood(const uint8_t* payload8,
                          uint8_t len,
                          uint8_t* outPrimaryCalc)
{
    if (outPrimaryCalc != nullptr) {
        *outPrimaryCalc = 0U;
    }
    if (payload8 == nullptr || len < 8U) {
        return false;
    }

    const uint8_t crcRx = payload8[7];
    const uint8_t crcFixed = compute1daWireCrcFixed(payload8);

    if (outPrimaryCalc != nullptr) {
        *outPrimaryCalc = crcFixed;
    }

    // Runtime validator intentionally follows strict shared LeafCRC path.
    return (crcRx == crcFixed);
}

CanHAL s_canHal;
Config s_config;
bool s_configured = false;
LeafInvFeedback s_feedback{};
Stats s_stats;

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
    static uint32_t lastCanStatMs = 0U;

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

    if ((nowMs - lastCanStatMs) >= 2000U) {
        lastCanStatMs = nowMs;
        Serial.printf("[CAN-STAT] ready=%u state=%u rx_total=%lu queued=%lu missed=%lu overrun=%lu\n",
                      static_cast<unsigned>(s_stats.ready ? 1U : 0U),
                      static_cast<unsigned>(twaiStatus.state),
                      static_cast<unsigned long>(s_stats.rxFrames),
                      static_cast<unsigned long>(s_stats.twaiRxQueued),
                      static_cast<unsigned long>(s_stats.twaiRxMissed),
                      static_cast<unsigned long>(s_stats.twaiRxOverrun));
    }

    if (twaiStatus.state == TWAI_STATE_BUS_OFF) {
        handleDriverFault(nowMs, true);
        return;
    }
    if (twaiStatus.state == TWAI_STATE_STOPPED) {
        handleDriverFault(nowMs, false);
        return;
    }

    // Drain the RX queue. Only 0x1DA (motor feedback, CRC-validated) and
    // 0x55A (temperatures) are decoded; every other CAN ID is ignored.
    for (uint8_t i = 0; i < s_config.maxFramesPerPoll; ++i) {
        uint32_t id = 0;
        uint8_t len = 0;
        uint8_t data[8] = {0};
        bool isExtended = false;
        if (!s_canHal.receive(id, data, len, isExtended)) {
            break;
        }

        if (id != 0x1DAU && id != 0x55AU) {
            continue;
        }

        twai_message_t msg = {};
        msg.identifier = id;
        msg.data_length_code = len;
        memcpy(msg.data, data, len);

        if (id == 0x1DAU) {
            uint8_t frame1daWireCrcCalc = 0U;
            const bool frame1daWireCrcOk = (len >= 8U) &&
                                           is1daWireCrcKnownGood(data, len, &frame1daWireCrcCalc);

            ++s_stats.rx1daFrames;
            s_stats.last1daMs = nowMs;
            s_stats.last1daLen = len;
            memset(s_stats.last1daData, 0, sizeof(s_stats.last1daData));
            memcpy(s_stats.last1daData, data, len);

            if (len >= 8U) {
                s_stats.last1daWireCrcOk = frame1daWireCrcOk ? 1 : 0;
                s_stats.last1daWireCrcCalc = frame1daWireCrcCalc;
                if (frame1daWireCrcOk) {
                    ++s_stats.rx1daCrcOkFrames;
                    ++s_stats.rx1daWireCrcOkFrames;
                } else {
                    ++s_stats.rx1daCrcBadFrames;
                    ++s_stats.rx1daWireCrcBadFrames;
                }
            } else {
                s_stats.last1daWireCrcOk = -1;
                s_stats.last1daWireCrcCalc = 0U;
            }

            // Bad-CRC 0x1DA frames are discarded; only validated frames update feedback.
            if (frame1daWireCrcOk) {
                LeafCan::decodeFrame(msg, s_feedback, nowMs);
            }
            ++s_stats.rxLeafFrames;
        } else {
            // 0x55A: temperatures only, no CRC gate defined for this frame.
            ++s_stats.rx55aFrames;
            s_stats.last55aMs = nowMs;
            s_stats.last55aLen = len;
            memset(s_stats.last55aData, 0, sizeof(s_stats.last55aData));
            memcpy(s_stats.last55aData, data, len);

            LeafCan::decodeFrame(msg, s_feedback, nowMs);
            ++s_stats.rxLeafFrames;
        }

        s_stats.lastRxMs = nowMs;
        ++s_stats.rxFrames;
    }
}

bool send(uint32_t id, const uint8_t* data, uint8_t len)
{
    if (!s_stats.ready) {
        return false;
    }

    return s_canHal.send(id, data, len);
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
