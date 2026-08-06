#include "CANBus.h"

#include <string.h>
#include "HardwareSerial.h"

#include "CanHAL.h"

#ifndef METASENSE_CAN_LOG_KEY_FRAMES
#define METASENSE_CAN_LOG_KEY_FRAMES 0
#endif

#ifndef METASENSE_CAN_LOG_ALL_FRAMES
#define METASENSE_CAN_LOG_ALL_FRAMES 0
#endif

#ifndef METASENSE_CAN_LOG_11A_CHANGES
#define METASENSE_CAN_LOG_11A_CHANGES 0
#endif

#ifndef METASENSE_CAN_ID_SCAN
#define METASENSE_CAN_ID_SCAN 1
#endif

namespace MetaSense::CANBus {

namespace {

CanHAL s_canHal;
Config s_config;
bool s_configured = false;
LeafInvFeedback s_feedback{};
Stats s_stats;

#if METASENSE_CAN_ID_SCAN
struct UnknownIdScanEntry {
    uint32_t id = 0xFFFFFFFFU;
    uint32_t frames = 0U;
    uint32_t changes = 0U;
    uint32_t extFrames = 0U;
    uint32_t lastMs = 0U;
    uint8_t lastLen = 0U;
    uint8_t lastData[8] = {0U};
};

constexpr size_t kUnknownIdScanSlots = 20U;
UnknownIdScanEntry s_unknownIdScan[kUnknownIdScanSlots];

static bool isTempTripletPlausible(const uint8_t* d, uint8_t start)
{
    const float t0 = static_cast<float>(d[start + 0U]) - 40.0f;
    const float t1 = static_cast<float>(d[start + 1U]) - 40.0f;
    const float t2 = static_cast<float>(d[start + 2U]) - 40.0f;
    return (t0 > -35.0f && t0 < 180.0f) &&
           (t1 > -35.0f && t1 < 180.0f) &&
           (t2 > -35.0f && t2 < 180.0f);
}

static void noteUnknownId(uint32_t id, const uint8_t* data, uint8_t len, uint32_t nowMs, bool isExtended)
{
    // Exclude only strict accepted IDs; keep everything else visible for hunt.
    if (id == 0x1DAU || id == 0x55AU) {
        return;
    }

    size_t slot = kUnknownIdScanSlots;
    size_t freeSlot = kUnknownIdScanSlots;
    size_t minSlot = 0U;
    uint32_t minFrames = 0xFFFFFFFFU;

    for (size_t i = 0; i < kUnknownIdScanSlots; ++i) {
        if (s_unknownIdScan[i].frames > 0U && s_unknownIdScan[i].id == id) {
            slot = i;
            break;
        }
        if (freeSlot == kUnknownIdScanSlots && s_unknownIdScan[i].frames == 0U) {
            freeSlot = i;
        }
        if (s_unknownIdScan[i].frames < minFrames) {
            minFrames = s_unknownIdScan[i].frames;
            minSlot = i;
        }
    }

    if (slot == kUnknownIdScanSlots) {
        slot = (freeSlot != kUnknownIdScanSlots) ? freeSlot : minSlot;
        s_unknownIdScan[slot] = UnknownIdScanEntry{};
        s_unknownIdScan[slot].id = id;
    }

    UnknownIdScanEntry& entry = s_unknownIdScan[slot];
    if (entry.frames > 0U && (entry.lastLen != len || memcmp(entry.lastData, data, len) != 0)) {
        ++entry.changes;
    }
    ++entry.frames;
    if (isExtended) {
        ++entry.extFrames;
    }
    entry.lastMs = nowMs;
    entry.lastLen = len;
    memset(entry.lastData, 0, sizeof(entry.lastData));
    memcpy(entry.lastData, data, len);
}

static void printUnknownIdScan(uint32_t nowMs)
{
    uint64_t best[kUnknownIdScanSlots] = {0U};
    size_t bestIdx[kUnknownIdScanSlots] = {0U};
    size_t bestCount = 0U;

    for (size_t i = 0; i < kUnknownIdScanSlots; ++i) {
        const uint32_t frames = s_unknownIdScan[i].frames;
        if (frames == 0U) {
            continue;
        }
        const uint64_t score = (static_cast<uint64_t>(s_unknownIdScan[i].changes) << 32) |
                               static_cast<uint64_t>(frames);
        size_t pos = bestCount;
        while (pos > 0U && score > best[pos - 1U]) {
            if (pos < kUnknownIdScanSlots) {
                best[pos] = best[pos - 1U];
                bestIdx[pos] = bestIdx[pos - 1U];
            }
            --pos;
        }
        if (pos < kUnknownIdScanSlots) {
            best[pos] = score;
            bestIdx[pos] = i;
            if (bestCount < kUnknownIdScanSlots) {
                ++bestCount;
            }
        }
    }

    const size_t toPrint = (bestCount < 6U) ? bestCount : 6U;
    if (toPrint == 0U) {
        Serial.printf("[CAN-ID-HUNT] no_candidates unknown_total=%lu\n",
                      static_cast<unsigned long>(s_stats.rxUnknownFrames));
        return;
    }

    for (size_t i = 0; i < toPrint; ++i) {
        const UnknownIdScanEntry& e = s_unknownIdScan[bestIdx[i]];
        char tempHint[24] = "none";
        if (e.lastLen >= 3U && isTempTripletPlausible(e.lastData, 0U)) {
            strcpy(tempHint, "off0");
        }
        if (e.lastLen >= 4U && isTempTripletPlausible(e.lastData, 1U)) {
            if (strcmp(tempHint, "none") == 0) {
                strcpy(tempHint, "off1");
            } else {
                strcat(tempHint, "+1");
            }
        }

        Serial.printf("[CAN-ID-HUNT] id=0x%03lX frames=%lu changes=%lu ext=%lu age=%lu len=%u data=%02X %02X %02X %02X temp_hint=%s\n",
                      static_cast<unsigned long>(e.id),
                      static_cast<unsigned long>(e.frames),
                  static_cast<unsigned long>(e.changes),
                  static_cast<unsigned long>(e.extFrames),
                      static_cast<unsigned long>(nowMs - e.lastMs),
                      static_cast<unsigned>(e.lastLen),
                      static_cast<unsigned>(e.lastData[0]),
                      static_cast<unsigned>(e.lastData[1]),
                      static_cast<unsigned>(e.lastData[2]),
                      static_cast<unsigned>(e.lastData[3]),
                      tempHint);
    }
}
#endif

bool isAcceptedLeafId(uint32_t id)
{
    return id == 0x1DAU || id == 0x55AU;
}

uint8_t changedMask(const uint8_t* prev, uint8_t prevLen, const uint8_t* curr, uint8_t currLen)
{
    if (currLen == 0U) {
        return 0U;
    }

    if (prevLen != currLen) {
        return (currLen >= 8U) ? 0xFFU : static_cast<uint8_t>((1U << currLen) - 1U);
    }

    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < currLen; ++i) {
        if (prev[i] != curr[i]) {
            mask |= static_cast<uint8_t>(1U << i);
        }
    }
    return mask;
}

uint32_t normalizeLeafIdForDecode(uint32_t id)
{
    // Strict mode: decode only exact on-bus IDs (no alias/sibling normalization).
    return id;
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
    static uint32_t lastRateMs = 0;
    static uint32_t lastUnknownScanMs = 0;
    static uint32_t lastUnknownAtScan = 0;
    static uint32_t lastRx1daFrames = 0;
    static uint32_t lastRx1d4CmdFrames = 0;
    static uint32_t lastRx1dcFrames = 0;
    static uint32_t lastRx1dbFrames = 0;
    static uint32_t lastRx11aFrames = 0;
    static uint32_t lastRxUnknownFrames = 0;
    static uint32_t lastRxStdFrames = 0;
    static uint32_t lastRxExtFrames = 0;
    static uint32_t last11aChangeLogMs = 0;
    const uint32_t DIAG_PERIOD_MS = 5000;
    const uint32_t RATE_PERIOD_MS = 2000;

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
        bool isExtended = false;
        if (!s_canHal.receive(id, data, len, isExtended)) {
            break;
        }
        
        if (isExtended) {
            ++s_stats.rxExtFrames;
            ++extFramesThisPoll;
        } else {
            ++s_stats.rxStdFrames;
        }

        ++framesThisPoll;

        const uint32_t decodeId = normalizeLeafIdForDecode(id);

        twai_message_t msg = {};
        msg.identifier = decodeId;
        msg.data_length_code = len;
        memcpy(msg.data, data, len);

        if (decodeId == 0x55AU) {
            ++s_stats.rx55aFrames;
            s_stats.last55aMs = nowMs;
            s_stats.last55aLen = len;
            memset(s_stats.last55aData, 0, sizeof(s_stats.last55aData));
            memcpy(s_stats.last55aData, data, len);
        }
        if (id == 0x1D4U) {
            ++s_stats.rx1d4SniffFrames;
            s_stats.last1d4SniffMs = nowMs;
            s_stats.last1d4SniffLen = len;
            memset(s_stats.last1d4SniffData, 0, sizeof(s_stats.last1d4SniffData));
            memcpy(s_stats.last1d4SniffData, data, len);
            ++s_stats.rx1d4CmdFrames;
            s_stats.last1d4CmdMs = nowMs;
            s_stats.last1d4CmdLen = len;
            memset(s_stats.last1d4CmdData, 0, sizeof(s_stats.last1d4CmdData));
            memcpy(s_stats.last1d4CmdData, data, len);
        }

        if (isAcceptedLeafId(decodeId)) {
            LeafCan::decodeFrame(msg, s_feedback, nowMs);
            ++s_stats.rxLeafFrames;
            if (decodeId == 0x1DAU) {
                ++s_stats.rx1daFrames;
                s_stats.last1daMs = nowMs;
                s_stats.last1daLen = len;
                memset(s_stats.last1daData, 0, sizeof(s_stats.last1daData));
                memcpy(s_stats.last1daData, data, len);
            } else if (decodeId == 0x1DCU) {
                ++s_stats.rx1dcFrames;
            }
        } else {
            ++s_stats.rxUnknownFrames;
#if METASENSE_CAN_ID_SCAN
            noteUnknownId(id, data, len, nowMs, isExtended);
#endif
            if (id == 0x1DBU) {
                ++s_stats.rx1dbFrames;
            }
            if (id == 0x11AU) {
                const uint8_t deltaMask = changedMask(s_stats.last11aData,
                                                      s_stats.last11aLen,
                                                      data,
                                                      len);
                s_stats.last11aChangeMask = deltaMask;
                if (s_stats.rx11aFrames > 0U && deltaMask != 0U) {
                    ++s_stats.rx11aChanges;
                }
                s_stats.agg11aChangeMask |= deltaMask;
                for (uint8_t b = 0U; b < len && b < 8U; ++b) {
                    if ((deltaMask & static_cast<uint8_t>(1U << b)) != 0U) {
                        ++s_stats.byteChg11a[b];
                    }
                }
#if METASENSE_CAN_LOG_11A_CHANGES
                if (deltaMask != 0U && (last11aChangeLogMs == 0U || (nowMs - last11aChangeLogMs) >= 100U)) {
                    Serial.printf("[CAN-11A-CHG] n=%lu chg=%lu mask=0x%02X data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                                  static_cast<unsigned long>(s_stats.rx11aFrames + 1U),
                                  static_cast<unsigned long>(s_stats.rx11aChanges),
                                  static_cast<unsigned>(deltaMask),
                                  static_cast<unsigned>(data[0]),
                                  static_cast<unsigned>(data[1]),
                                  static_cast<unsigned>(data[2]),
                                  static_cast<unsigned>(data[3]),
                                  static_cast<unsigned>(data[4]),
                                  static_cast<unsigned>(data[5]),
                                  static_cast<unsigned>(data[6]),
                                  static_cast<unsigned>(data[7]));
                    last11aChangeLogMs = nowMs;
                }
#endif
                ++s_stats.rx11aFrames;
                s_stats.last11aMs = nowMs;
                s_stats.last11aLen = len;
                memset(s_stats.last11aData, 0, sizeof(s_stats.last11aData));
                memcpy(s_stats.last11aData, data, len);
            }
            s_stats.lastUnknownMs = nowMs;
            s_stats.lastUnknownId = id;
            s_stats.lastUnknownLen = len;
            memset(s_stats.lastUnknownData, 0, sizeof(s_stats.lastUnknownData));
            memcpy(s_stats.lastUnknownData, data, len);
        }
        s_stats.lastRxMs = nowMs;
        s_stats.lastRxId = id;
        ++s_stats.rxFrames;
        
        // Optional raw passive sniff log for the two canonical frames of interest.
    #if METASENSE_CAN_LOG_KEY_FRAMES
        if (id == 0x1DAU || id == 0x55AU) {
            Serial.printf("[CAN-RX-RAW] id=0x%03lX len=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                  static_cast<unsigned long>(id),
                  static_cast<unsigned>(len),
                  static_cast<unsigned>(data[0]),
                  static_cast<unsigned>(data[1]),
                  static_cast<unsigned>(data[2]),
                  static_cast<unsigned>(data[3]),
                  static_cast<unsigned>(data[4]),
                  static_cast<unsigned>(data[5]),
                  static_cast<unsigned>(data[6]),
                  static_cast<unsigned>(data[7]));
        }
    #endif

        // Optional full-frame log.
    #if METASENSE_CAN_LOG_ALL_FRAMES
        Serial.printf("[CAN-FRAME-RX] id=0x%03lX len=%u data=%02X %02X %02X %02X\n",
                      static_cast<unsigned long>(id),
                      static_cast<unsigned>(len),
                      static_cast<unsigned>(data[0]),
                      static_cast<unsigned>(data[1]),
                      static_cast<unsigned>(data[2]),
                      static_cast<unsigned>(data[3]));
    #endif
        
    }
    
    if (lastDiagMs == 0U || (nowMs - lastDiagMs) >= DIAG_PERIOD_MS) {
        Serial.printf("[CAN-POLL-FRAME-COUNT] this_poll=%lu ext_frames=%lu\n", static_cast<unsigned long>(framesThisPoll), static_cast<unsigned long>(extFramesThisPoll));
    }

    if (lastRateMs == 0U || (nowMs - lastRateMs) >= RATE_PERIOD_MS) {
        const uint32_t dtMs = (lastRateMs == 0U) ? RATE_PERIOD_MS : (nowMs - lastRateMs);
        const uint32_t d1da = s_stats.rx1daFrames - lastRx1daFrames;
        const uint32_t d1d4cmd = s_stats.rx1d4CmdFrames - lastRx1d4CmdFrames;
        const uint32_t d1db = s_stats.rx1dbFrames - lastRx1dbFrames;
        const uint32_t d11a = s_stats.rx11aFrames - lastRx11aFrames;
        const uint32_t dunk = s_stats.rxUnknownFrames - lastRxUnknownFrames;
        const uint32_t dstd = s_stats.rxStdFrames - lastRxStdFrames;
        const uint32_t dext = s_stats.rxExtFrames - lastRxExtFrames;

        const float hz1da = (dtMs > 0U) ? (1000.0f * static_cast<float>(d1da) / static_cast<float>(dtMs)) : 0.0f;
        const float hz1d4cmd = (dtMs > 0U) ? (1000.0f * static_cast<float>(d1d4cmd) / static_cast<float>(dtMs)) : 0.0f;
        const float hz1db = (dtMs > 0U) ? (1000.0f * static_cast<float>(d1db) / static_cast<float>(dtMs)) : 0.0f;
        const float hz11a = (dtMs > 0U) ? (1000.0f * static_cast<float>(d11a) / static_cast<float>(dtMs)) : 0.0f;
        const float hzStd = (dtMs > 0U) ? (1000.0f * static_cast<float>(dstd) / static_cast<float>(dtMs)) : 0.0f;
        const float hzExt = (dtMs > 0U) ? (1000.0f * static_cast<float>(dext) / static_cast<float>(dtMs)) : 0.0f;

        Serial.printf("[CAN-RATE] std=%lu(%.1fHz) ext=%lu(%.1fHz) 1DA=%lu(%.1fHz) 1D4cmd=%lu(%.1fHz) 1DB=%lu(%.1fHz) legacy11A=%lu(%.1fHz) unk=%lu\n",
                      static_cast<unsigned long>(s_stats.rxStdFrames), hzStd,
                      static_cast<unsigned long>(s_stats.rxExtFrames), hzExt,
                      static_cast<unsigned long>(s_stats.rx1daFrames), hz1da,
                      static_cast<unsigned long>(s_stats.rx1d4CmdFrames), hz1d4cmd,
                  static_cast<unsigned long>(s_stats.rx1dbFrames), hz1db,
                  static_cast<unsigned long>(s_stats.rx11aFrames), hz11a,
                      static_cast<unsigned long>(s_stats.rxUnknownFrames));

        lastRateMs = nowMs;
        lastRx1daFrames = s_stats.rx1daFrames;
        lastRx1d4CmdFrames = s_stats.rx1d4CmdFrames;
        lastRx1dbFrames = s_stats.rx1dbFrames;
        lastRx11aFrames = s_stats.rx11aFrames;
        lastRxUnknownFrames = s_stats.rxUnknownFrames;
        lastRxStdFrames = s_stats.rxStdFrames;
        lastRxExtFrames = s_stats.rxExtFrames;
        (void)dunk;
    }

#if METASENSE_CAN_ID_SCAN
    if (lastUnknownScanMs == 0U || (nowMs - lastUnknownScanMs) >= 5000U) {
        if (s_stats.rxUnknownFrames != lastUnknownAtScan) {
            printUnknownIdScan(nowMs);
            lastUnknownAtScan = s_stats.rxUnknownFrames;
        }
        lastUnknownScanMs = nowMs;
    }
#endif
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
        if (id == 0x1D4U && data != nullptr) {
            ++s_stats.tx1d4Frames;
            s_stats.last1d4TxMs = millis();
            s_stats.last1d4TxLen = (len <= sizeof(s_stats.last1d4TxData))
                ? len
                : static_cast<uint8_t>(sizeof(s_stats.last1d4TxData));
            memset(s_stats.last1d4TxData, 0, sizeof(s_stats.last1d4TxData));
            memcpy(s_stats.last1d4TxData, data, s_stats.last1d4TxLen);

            // Some installations do not receive local TX loopback. Mirror last
            // transmitted 0x1D4 command into monitor stats so UI CRC/bytes stay visible.
            ++s_stats.rx1d4CmdFrames;
            s_stats.last1d4CmdMs = millis();
            s_stats.last1d4CmdLen = (len <= sizeof(s_stats.last1d4CmdData))
                ? len
                : static_cast<uint8_t>(sizeof(s_stats.last1d4CmdData));
            memset(s_stats.last1d4CmdData, 0, sizeof(s_stats.last1d4CmdData));
            memcpy(s_stats.last1d4CmdData, data, s_stats.last1d4CmdLen);
        }
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
