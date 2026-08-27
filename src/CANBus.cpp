#include "CANBus.h"

#include <string.h>
#include <LittleFS.h>
#include "HardwareSerial.h"
#include "HardwareOutputStateMachine.h"
#include "LeafCrc.h"
#include "globals.h"
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

#if METASENSE_LEAF_1D4_RINGBUF_ENABLE
constexpr uint8_t kLeaf1d4RingSize = static_cast<uint8_t>(METASENSE_LEAF_1D4_RINGBUF_SIZE);
static uint8_t s_leaf1d4Ring[kLeaf1d4RingSize][8] = {{0U}};
static uint8_t s_leaf1d4RingLen[kLeaf1d4RingSize] = {0U};
static uint8_t s_leaf1d4RingHead = 0U;  // Next write slot.
static uint8_t s_leaf1d4RingCount = 0U;

static void resetLeaf1d4Ring()
{
    memset(s_leaf1d4Ring, 0, sizeof(s_leaf1d4Ring));
    memset(s_leaf1d4RingLen, 0, sizeof(s_leaf1d4RingLen));
    s_leaf1d4RingHead = 0U;
    s_leaf1d4RingCount = 0U;
}

static void pushLeaf1d4Ring(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || kLeaf1d4RingSize == 0U) {
        return;
    }

    const uint8_t slot = s_leaf1d4RingHead;
    const uint8_t copyLen = (len <= 8U) ? len : 8U;

    memset(s_leaf1d4Ring[slot], 0, sizeof(s_leaf1d4Ring[slot]));
    memcpy(s_leaf1d4Ring[slot], data, copyLen);
    s_leaf1d4RingLen[slot] = copyLen;

    s_leaf1d4RingHead = static_cast<uint8_t>((s_leaf1d4RingHead + 1U) % kLeaf1d4RingSize);
    if (s_leaf1d4RingCount < kLeaf1d4RingSize) {
        ++s_leaf1d4RingCount;
    }
}

static bool readLeaf1d4Ring(uint8_t ageFromNewest, uint8_t* outData, uint8_t* outLen)
{
    if (s_leaf1d4RingCount == 0U || outData == nullptr || ageFromNewest >= s_leaf1d4RingCount) {
        return false;
    }

    const uint8_t newestSlot = static_cast<uint8_t>((s_leaf1d4RingHead + kLeaf1d4RingSize - 1U) % kLeaf1d4RingSize);
    const uint8_t slot = static_cast<uint8_t>((newestSlot + kLeaf1d4RingSize - (ageFromNewest % kLeaf1d4RingSize)) % kLeaf1d4RingSize);
    memcpy(outData, s_leaf1d4Ring[slot], 8U);
    if (outLen != nullptr) {
        *outLen = s_leaf1d4RingLen[slot];
    }
    return true;
}
#endif

bool isAcceptedLeafId(uint32_t id)
{
    return id == 0x1DAU || id == 0x55AU;
}

bool isRxIdExcluded(uint32_t id)
{
    // Closed RX policy: explicitly ignore command/echo families.
    if (id == 0x11AU) {
        return true;
    }
    return id == 0x1D4U;
}

static void logRxFrameOneLine(uint32_t id, const uint8_t* data, uint8_t len, bool isExtended)
{
    if (data == nullptr) {
        return;
    }

    const uint8_t safeLen = (len <= 8U) ? len : 8U;
    Serial.printf("[CAN-RX] id=0x%03lX ext=%u len=%u data=",
                  static_cast<unsigned long>(id),
                  static_cast<unsigned>(isExtended ? 1U : 0U),
                  static_cast<unsigned>(safeLen));
    for (uint8_t b = 0U; b < safeLen; ++b) {
        Serial.printf("%02X", static_cast<unsigned>(data[b]));
        if (b + 1U < safeLen) {
            Serial.print(' ');
        }
    }
    Serial.println();
}

static void logUnknownRxFrameOneLine(uint32_t id, const uint8_t* data, uint8_t len, bool isExtended)
{
    if (data == nullptr) {
        return;
    }

    const uint8_t safeLen = (len <= 8U) ? len : 8U;
    Serial.printf("[CAN-UNK] id=0x%03lX ext=%u len=%u data=",
                  static_cast<unsigned long>(id),
                  static_cast<unsigned>(isExtended ? 1U : 0U),
                  static_cast<unsigned>(safeLen));
    for (uint8_t b = 0U; b < safeLen; ++b) {
        Serial.printf("%02X", static_cast<unsigned>(data[b]));
        if (b + 1U < safeLen) {
            Serial.print(' ');
        }
    }
    Serial.println();
}

static void logTargetRxFrame(uint32_t id, const uint8_t* data, uint8_t len, bool isExtended)
{
    if (data == nullptr) {
        return;
    }
    if (id != 0x11AU) {
        return;
    }

    const uint8_t safeLen = (len <= 8U) ? len : 8U;
    Serial.printf("[CAN-TARGET] id=0x%03lX ext=%u len=%u data=",
                  static_cast<unsigned long>(id),
                  static_cast<unsigned>(isExtended ? 1U : 0U),
                  static_cast<unsigned>(safeLen));
    for (uint8_t b = 0U; b < safeLen; ++b) {
        Serial.printf("%02X", static_cast<unsigned>(data[b]));
        if (b + 1U < safeLen) {
            Serial.print(' ');
        }
    }
    Serial.println();

    if (id == 0x11AU && safeLen >= 8U) {
        static bool s_haveLast11aDecoded = false;
        static uint8_t s_last11aDecoded[8] = {0U};

        const bool payloadChanged = !s_haveLast11aDecoded || (memcmp(s_last11aDecoded, data, 8U) != 0);
        if (!payloadChanged) {
            return;
        }

        const bool hadPrev = s_haveLast11aDecoded;
        const uint8_t prevMux = hadPrev ? s_last11aDecoded[6] : 0U;
        const uint8_t prevStartup = hadPrev ? s_last11aDecoded[7] : 0U;
        const uint8_t prevHeartbeat = hadPrev ? s_last11aDecoded[3] : 0U;

        memcpy(s_last11aDecoded, data, 8U);
        s_haveLast11aDecoded = true;

        const uint8_t joystickGearPosition = static_cast<uint8_t>((data[0] >> 4U) & 0x0FU);   // 4|4@1+
        const uint8_t ecoSelected = static_cast<uint8_t>((data[1] >> 4U) & 0x01U);             // 12|1@1+
        const uint8_t carOnOffStatus = static_cast<uint8_t>((data[1] >> 5U) & 0x07U);           // 13|3@1+
        const uint8_t steeringWheelButton = data[2];                                             // 16|8@1+
        const uint8_t heartbeatVcm = data[3];                                                    // 24|8@1+
        const uint8_t unknown11a4 = data[4];                                                     // 32|8@1+ (muxed)
        const uint8_t multiplexor = data[6];                                                     // 48|8@1+
        const uint8_t startupData = data[7];                                                     // 56|8@1+ (muxed)

        Serial.printf("[CAN-11A] gear=%u car_onoff=%u eco=%u btn=0x%02X hb=%u mux=%u u4=0x%02X startup=0x%02X\n",
                      static_cast<unsigned>(joystickGearPosition),
                      static_cast<unsigned>(carOnOffStatus),
                      static_cast<unsigned>(ecoSelected),
                      static_cast<unsigned>(steeringWheelButton),
                      static_cast<unsigned>(heartbeatVcm),
                      static_cast<unsigned>(multiplexor),
                      static_cast<unsigned>(unknown11a4),
                      static_cast<unsigned>(startupData));
        if (hadPrev && (prevMux != multiplexor || prevStartup != startupData || prevHeartbeat != heartbeatVcm)) {
            Serial.printf("[CAN-11A-SIG] mux=%u>%u startup=%02X>%02X hb=%02X>%02X\n",
                          static_cast<unsigned>(prevMux),
                          static_cast<unsigned>(multiplexor),
                          static_cast<unsigned>(prevStartup),
                          static_cast<unsigned>(startupData),
                          static_cast<unsigned>(prevHeartbeat),
                          static_cast<unsigned>(heartbeatVcm));
        }
    }
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
        } else {
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
#if METASENSE_LEAF_1D4_RINGBUF_ENABLE
    resetLeaf1d4Ring();
#endif
}

void poll(uint32_t nowMs)
{
    static uint32_t lastPollHeartMs = 0U;
    static uint32_t lastDiagMs = 0;
    static uint32_t lastCanStatMs = 0U;
#if !METASENSE_CAN_RX_ONE_LINE_LOG
    if ((nowMs - lastPollHeartMs) >= 2000U) {
        lastPollHeartMs = nowMs;
        Serial.printf("[CAN-POLL-HEART] now=%lu configured=%d ready=%d\n",
                      static_cast<unsigned long>(nowMs),
                      static_cast<int>(s_configured),
                      static_cast<int>(s_stats.ready));
        Serial0.printf("[CAN-POLL-HEART] now=%lu configured=%d ready=%d\n",
                       static_cast<unsigned long>(nowMs),
                       static_cast<int>(s_configured),
                       static_cast<int>(s_stats.ready));
    }
#endif

    if (!ensureReady(nowMs)) {
        return;
    }
    static uint32_t lastRateMs = 0;
    static uint32_t lastUnknownScanMs = 0;
    static uint32_t lastUnknownAtScan = 0;
    static uint32_t lastSniffSummaryMs = 0;
    static uint32_t lastSniffPrintMs = 0;
    static uint32_t lastSniffSnapshotCount = 0;
    static uint32_t lastRx1daFrames = 0;
    static uint32_t lastRx1d4CmdFrames = 0;
    static uint32_t lastRx1dcFrames = 0;
    static uint32_t lastRx1dbFrames = 0;
    static uint32_t lastRx11aFrames = 0;
    static uint32_t lastRxUnknownFrames = 0;
    static uint32_t lastRxStdFrames = 0;
    static uint32_t lastRxExtFrames = 0;
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

    if ((nowMs - lastCanStatMs) >= 2000U) {
        lastCanStatMs = nowMs;
        Serial.printf("[CAN-STAT] ready=%u state=%u rx_total=%lu last_id=0x%03lX queued=%lu missed=%lu overrun=%lu\n",
                      static_cast<unsigned>(s_stats.ready ? 1U : 0U),
                      static_cast<unsigned>(twaiStatus.state),
                      static_cast<unsigned long>(s_stats.rxFrames),
                      static_cast<unsigned long>(s_stats.lastRxId),
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
    
    // Periodic diagnostic: show queue status
#if !METASENSE_CAN_SNIFF_ONLY
#if !METASENSE_CAN_RX_ONE_LINE_LOG
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
#endif
#else
#if !METASENSE_CAN_RX_ONE_LINE_LOG
    if (lastDiagMs == 0U || (nowMs - lastDiagMs) >= DIAG_PERIOD_MS) {
        lastDiagMs = nowMs;
        const uint32_t lastRxAgeMs = (s_stats.lastRxMs == 0U) ? 0U : (nowMs - s_stats.lastRxMs);
        Serial.printf("[CAN-SNIFF-DIAG] state=%u queued=%lu missed=%lu overrun=%lu arb_lost=%lu bus_err=%lu rx_total=%lu std=%lu ext=%lu leaf=%lu 1da=%lu 55a=%lu 11a=%lu unk=%lu last_id=0x%03lX last_age=%lu\n",
                      static_cast<unsigned>(twaiStatus.state),
                      static_cast<unsigned long>(twaiStatus.msgs_to_rx),
                      static_cast<unsigned long>(twaiStatus.rx_missed_count),
                      static_cast<unsigned long>(twaiStatus.rx_overrun_count),
                      static_cast<unsigned long>(twaiStatus.arb_lost_count),
                      static_cast<unsigned long>(twaiStatus.bus_error_count),
                      static_cast<unsigned long>(s_stats.rxFrames),
                      static_cast<unsigned long>(s_stats.rxStdFrames),
                      static_cast<unsigned long>(s_stats.rxExtFrames),
                      static_cast<unsigned long>(s_stats.rxLeafFrames),
                      static_cast<unsigned long>(s_stats.rx1daFrames),
                      static_cast<unsigned long>(s_stats.rx55aFrames),
                      static_cast<unsigned long>(s_stats.rx11aFrames),
                      static_cast<unsigned long>(s_stats.rxUnknownFrames),
                      static_cast<unsigned long>(s_stats.lastRxId),
                      static_cast<unsigned long>(lastRxAgeMs));
    }
#endif
#endif

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

        if (isRxIdExcluded(id)) {
            // Do not decode, count as unknown, or surface excluded IDs.
            continue;
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

        int8_t frame1daWireCrcOk = -1;
        uint8_t frame1daWireCrcCalc = 0U;
        if (decodeId == 0x1DAU && len >= 8U) {
            frame1daWireCrcOk = is1daWireCrcKnownGood(data, len, &frame1daWireCrcCalc) ? 1 : 0;
        }

        if (decodeId == 0x55AU) {
            ++s_stats.rx55aFrames;
            s_stats.last55aMs = nowMs;
            s_stats.last55aLen = len;
            memset(s_stats.last55aData, 0, sizeof(s_stats.last55aData));
            memcpy(s_stats.last55aData, data, len);
        }
        if (id == 0x1D4U) {
    #if METASENSE_LEAF_1D4_RINGBUF_ENABLE
            pushLeaf1d4Ring(data, len);
    #endif
    #if METASENSE_LEAF_1D4_SNIFF_RX_ENABLED
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
    #endif
        }

        if (isAcceptedLeafId(decodeId)) {
            // Decode accepted telemetry in both normal and sniff-only modes so
            // monitor fields (e.g. 0x55A temperatures) stay live.
            // Exception: 0x1DA is CRC-gated; BAD frames are discarded.
            const bool allowDecode = (decodeId != 0x1DAU) || (frame1daWireCrcOk == 1);
            if (allowDecode) {
                LeafCan::decodeFrame(msg, s_feedback, nowMs);
            }
            ++s_stats.rxLeafFrames;
            if (decodeId == 0x1DAU) {
                ++s_stats.rx1daFrames;
                s_stats.last1daMs = nowMs;
                s_stats.last1daLen = len;
                memset(s_stats.last1daData, 0, sizeof(s_stats.last1daData));
                memcpy(s_stats.last1daData, data, len);
                if (len >= 8U) {
                    s_stats.last1daWireCrcOk = frame1daWireCrcOk;
                    s_stats.last1daWireCrcCalc = frame1daWireCrcCalc;
                    if (s_stats.last1daWireCrcOk == 1) {
                        ++s_stats.rx1daWireCrcOkFrames;
                    } else {
                        ++s_stats.rx1daWireCrcBadFrames;
                        static uint32_t s_last1daBadLogMs = 0U;
                        if ((nowMs - s_last1daBadLogMs) >= 250U) {
                            s_last1daBadLogMs = nowMs;
                            const uint8_t crcRx = data[7];
                            const uint8_t crcCalc = s_stats.last1daWireCrcCalc;
                            const uint8_t clock = static_cast<uint8_t>(data[6] & 0x03U);
                            Serial.printf("[1DA-CRC-BAD] n=%lu clk=%u rx=0x%02X calc=0x%02X data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                                          static_cast<unsigned long>(s_stats.rx1daWireCrcBadFrames),
                                          static_cast<unsigned>(clock),
                                          static_cast<unsigned>(crcRx),
                                          static_cast<unsigned>(crcCalc),
                                          static_cast<unsigned>(data[0]),
                                          static_cast<unsigned>(data[1]),
                                          static_cast<unsigned>(data[2]),
                                          static_cast<unsigned>(data[3]),
                                          static_cast<unsigned>(data[4]),
                                          static_cast<unsigned>(data[5]),
                                          static_cast<unsigned>(data[6]),
                                          static_cast<unsigned>(data[7]));
                            Serial0.printf("[1DA-CRC-BAD] n=%lu clk=%u rx=0x%02X calc=0x%02X data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                                           static_cast<unsigned long>(s_stats.rx1daWireCrcBadFrames),
                                           static_cast<unsigned>(clock),
                                           static_cast<unsigned>(crcRx),
                                           static_cast<unsigned>(crcCalc),
                                           static_cast<unsigned>(data[0]),
                                           static_cast<unsigned>(data[1]),
                                           static_cast<unsigned>(data[2]),
                                           static_cast<unsigned>(data[3]),
                                           static_cast<unsigned>(data[4]),
                                           static_cast<unsigned>(data[5]),
                                           static_cast<unsigned>(data[6]),
                                           static_cast<unsigned>(data[7]));
                        }
                    }
                } else {
                    s_stats.last1daWireCrcOk = -1;
                    s_stats.last1daWireCrcCalc = 0U;
                }
            } else if (decodeId == 0x1DCU) {
                ++s_stats.rx1dcFrames;
            }
        } else {
            ++s_stats.rxUnknownFrames;
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
                ++s_stats.rx11aFrames;
                s_stats.last11aMs = nowMs;
                s_stats.last11aLen = len;
                memset(s_stats.last11aData, 0, sizeof(s_stats.last11aData));
                memcpy(s_stats.last11aData, data, len);
            }
            if (id == 0x50BU) {
                ++s_stats.rx50bFrames;
                s_stats.last50bMs = nowMs;
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

        // Focused hunt: always surface these candidate IDs clearly.
        logTargetRxFrame(id, data, len, isExtended);

        // Unconditional raw frame dump for sniffing every frame on the bus.
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

        Serial.printf("[CAN-FRAME-RX] id=0x%03lX len=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
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

#if !METASENSE_CAN_SNIFF_ONLY
#if !METASENSE_CAN_RX_ONE_LINE_LOG
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
#endif
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
        if (id == 0x11AU && data != nullptr) {
            ++s_stats.tx11aFrames;
            s_stats.last11aTxMs = millis();
            s_stats.last11aTxLen = (len <= sizeof(s_stats.last11aTxData))
                ? len
                : static_cast<uint8_t>(sizeof(s_stats.last11aTxData));
            memset(s_stats.last11aTxData, 0, sizeof(s_stats.last11aTxData));
            memcpy(s_stats.last11aTxData, data, s_stats.last11aTxLen);
        }
        if (id == 0x1D4U && data != nullptr) {
            ++s_stats.tx1d4Frames;
            s_stats.last1d4TxMs = millis();
            s_stats.last1d4TxLen = (len <= sizeof(s_stats.last1d4TxData))
                ? len
                : static_cast<uint8_t>(sizeof(s_stats.last1d4TxData));
            memset(s_stats.last1d4TxData, 0, sizeof(s_stats.last1d4TxData));
            memcpy(s_stats.last1d4TxData, data, s_stats.last1d4TxLen);
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

StartupSniffStatus startupSniffStatus()
{
    StartupSniffStatus status;
    return status;
}

bool saveStartupSniffCaptureToFile()
{
    return false;
}

bool printStartupSniffCapture()
{
    return false;
}

uint8_t get1d4RingCount()
{
#if METASENSE_LEAF_1D4_RINGBUF_ENABLE
    return s_leaf1d4RingCount;
#else
    return 0U;
#endif
}

bool get1d4RingFrame(uint8_t ageFromNewest, uint8_t* outData, uint8_t* outLen)
{
#if METASENSE_LEAF_1D4_RINGBUF_ENABLE
    return readLeaf1d4Ring(ageFromNewest, outData, outLen);
#else
    (void)ageFromNewest;
    (void)outData;
    if (outLen != nullptr) {
        *outLen = 0U;
    }
    return false;
#endif
}

} // namespace MetaSense::CANBus
