#include "CANBus.h"

#include <string.h>
#include <LittleFS.h>
#include "HardwareSerial.h"
#include "HardwareOutputStateMachine.h"
#include "LeafCrc.h"
#include "globals.h"

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

#ifndef METASENSE_CAN_SNIFF_ONLY
#define METASENSE_CAN_SNIFF_ONLY 0
#endif

#ifndef METASENSE_CAN_FRAME_IDENTIFIER
#define METASENSE_CAN_FRAME_IDENTIFIER 0
#endif

#ifndef METASENSE_CAN_RX_ONE_LINE_LOG
#define METASENSE_CAN_RX_ONE_LINE_LOG 0
#endif

#ifndef METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
#define METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE 0
#endif

#ifndef METASENSE_STARTUP_SNIFF_CAPTURE_MS
#define METASENSE_STARTUP_SNIFF_CAPTURE_MS 2000U
#endif

#ifndef METASENSE_STARTUP_SNIFF_CAPTURE_MAX_FRAMES
#define METASENSE_STARTUP_SNIFF_CAPTURE_MAX_FRAMES 2048U
#endif

#ifndef METASENSE_STARTUP_SNIFF_ARM_DELAY_MS
#define METASENSE_STARTUP_SNIFF_ARM_DELAY_MS 0U
#endif

#ifndef METASENSE_STARTUP_SNIFF_START_ON_FIRST_11A
#define METASENSE_STARTUP_SNIFF_START_ON_FIRST_11A 0U
#endif

#ifndef METASENSE_STARTUP_SNIFF_REARM_ON_11A_GAP_MS
#define METASENSE_STARTUP_SNIFF_REARM_ON_11A_GAP_MS 0U
#endif

#ifndef METASENSE_STARTUP_SNIFF_DUMP_DELAY_MS
#define METASENSE_STARTUP_SNIFF_DUMP_DELAY_MS 30000U
#endif

#ifndef METASENSE_STARTUP_SNIFF_SAVE_PATH
#define METASENSE_STARTUP_SNIFF_SAVE_PATH "/captures/startup_11a.csv"
#endif

#ifndef METASENSE_LEAF_1D4_SNIFF_RX_ENABLED
#define METASENSE_LEAF_1D4_SNIFF_RX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_1D4_RINGBUF_ENABLE
#define METASENSE_LEAF_1D4_RINGBUF_ENABLE 0
#endif

#ifndef METASENSE_LEAF_1D4_RINGBUF_SIZE
#define METASENSE_LEAF_1D4_RINGBUF_SIZE 64U
#endif

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
    // DEFINITIVE: Use Nissan's poly 0x85 for 0x1DA validation (NOT poly 0x1D)
    // Nissan inverter broadcasts 0x1DA with this standard
    const uint8_t crcFixed = MetaSense::LeafCRC::computeExact1daWireCrc(0xDAU, payload8);

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
    if (id == 0x120U || id == 0x11AU) {
        return true;
    }
    // Exclude 0x1D4 unless sniffing
    return id == 0x1D4U;
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
#if METASENSE_CAN_SNIFF_ONLY
        Serial.printf("[CAN-SNIFF] passive-listen active (tx=%d rx=%d)\n",
                  s_config.txPin,
                  s_config.rxPin);
#else
        LeafCan::reset(s_feedback);
#endif
        } else {
    #if METASENSE_CAN_SNIFF_ONLY
        Serial.printf("[CAN-SNIFF-ERR] init failed (tx=%d rx=%d)\n",
                  s_config.txPin,
                  s_config.rxPin);
    #endif
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
#if METASENSE_CAN_ID_SCAN
    clearSniffIdEntries();
#endif
#if METASENSE_LEAF_1D4_RINGBUF_ENABLE
    resetLeaf1d4Ring();
#endif
#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
    resetStartupSniffCapture();
#endif
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

    // Track 0x1DA CRC errors for fault detection
    static uint32_t lastRateCheckMs = 0U;
    static uint32_t lastRx1daBadFramesAtCheck = 0U;
    
    for (uint8_t i = 0; i < s_config.maxFramesPerPoll; ++i) {
        uint32_t id = 0;
        uint8_t len = 0;
        uint8_t data[8] = {0};
        bool isExtended = false;
        if (!s_canHal.receive(id, data, len, isExtended)) {
            break;
        }

        // Skip excluded IDs (0x120, 0x11A, 0x1D4 unless sniffing)
        if (isRxIdExcluded(id)) {
            continue;
        }

        const uint32_t decodeId = normalizeLeafIdForDecode(id);

        // Check 0x1DA CRC (if present)
        int8_t frame1daWireCrcOk = -1;
        uint8_t frame1daWireCrcCalc = 0U;
        if (decodeId == 0x1DAU && len >= 8U) {
            frame1daWireCrcOk = is1daWireCrcKnownGood(data, len, &frame1daWireCrcCalc) ? 1 : 0;
        }

        // Store last frame data for TX echo capture
        if (id == 0x1D4U) {
            s_stats.last1d4CmdMs = nowMs;
            s_stats.last1d4CmdLen = len;
            memset(s_stats.last1d4CmdData, 0, sizeof(s_stats.last1d4CmdData));
            memcpy(s_stats.last1d4CmdData, data, len);
        }

        // Decode accepted Leaf frames
        if (isAcceptedLeafId(decodeId)) {
            // Now using unified Nissan 0x1D CRC for 0x1DA validation
            // Replaces incorrect 0x85 polynomial with standard 0x1D
            const bool allowDecode = (decodeId != 0x1DAU) || (frame1daWireCrcOk == 1);
            if (allowDecode) {
                twai_message_t msg = {};
                msg.identifier = decodeId;
                msg.data_length_code = len;
                memcpy(msg.data, data, len);
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
                    if (frame1daWireCrcOk == 1) {
                        ++s_stats.rx1daWireCrcOkFrames;
                    } else {
                        ++s_stats.rx1daWireCrcBadFrames;
                    }
                } else {
                    s_stats.last1daWireCrcOk = -1;
                    s_stats.last1daWireCrcCalc = 0U;
                }
            } else if (decodeId == 0x55AU) {
                ++s_stats.rx55aFrames;
                s_stats.last55aMs = nowMs;
                s_stats.last55aLen = len;
                memset(s_stats.last55aData, 0, sizeof(s_stats.last55aData));
                memcpy(s_stats.last55aData, data, len);
            }
        }

        s_stats.lastRxMs = nowMs;
        s_stats.lastRxId = id;
        ++s_stats.rxFrames;
    }

    // Check 0x1DA CRC error rate: fault if > 100/sec
    if (lastRateCheckMs == 0U || (nowMs - lastRateCheckMs) >= 1000U) {
        const uint32_t dtMs = (lastRateCheckMs == 0U) ? 1000U : (nowMs - lastRateCheckMs);
        const uint32_t d1daBad = s_stats.rx1daWireCrcBadFrames - lastRx1daBadFramesAtCheck;
        const float errorRatePerSec = (dtMs > 0U) 
            ? (1000.0f * static_cast<float>(d1daBad) / static_cast<float>(dtMs))
            : 0.0f;
        
        if (errorRatePerSec > 100.0f) {
            handleDriverFault(nowMs, false);
        }
        
        lastRateCheckMs = nowMs;
        lastRx1daBadFramesAtCheck = s_stats.rx1daWireCrcBadFrames;
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
#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
    status.enabled = true;
    status.active = s_startupSniffActive;
    status.done = s_startupSniffDone;
    status.dumped = s_startupSniffDumped;
    status.count = s_startupSniffCount;
    status.dropped = s_startupSniffDropped;
    status.armAtMs = s_startupSniffArmAtMs;
    status.last11aMs = s_startupSniffLast11aMs;
    status.dumpAtMs = s_startupSniffCompletedMs + METASENSE_STARTUP_SNIFF_DUMP_DELAY_MS;
#endif
    return status;
}

bool saveStartupSniffCaptureToFile()
{
#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
    if (!s_startupSniffDone || s_startupSniffDumped) {
        return false;
    }

    const uint32_t elapsedMs = (s_startupSniffCompletedMs >= s_startupSniffStartMs)
                                   ? (s_startupSniffCompletedMs - s_startupSniffStartMs)
                                   : 0U;

    LittleFS.mkdir("/captures");
    File file = LittleFS.open(METASENSE_STARTUP_SNIFF_SAVE_PATH, "w");
    if (!file) {
        Serial.printf("[STARTUP-SNIFF] save failed path=%s\n", METASENSE_STARTUP_SNIFF_SAVE_PATH);
        return false;
    }

    file.printf("meta,capture_ms,%lu,count,%u,dropped,%u\n",
                static_cast<unsigned long>(elapsedMs),
                static_cast<unsigned>(s_startupSniffCount),
                static_cast<unsigned>(s_startupSniffDropped));
    file.println("dt_ms,id_hex,id_dec,len,ext,gear,car_onoff,eco,btn,hb,mux,u4,startup,b0,b1,b2,b3,b4,b5,b6,b7");

    for (uint16_t i = 0U; i < s_startupSniffCount; ++i) {
        const StartupSniffFrame& frame = s_startupSniffFrames[i];
        uint8_t gear = 0U;
        uint8_t eco = 0U;
        uint8_t carOnOff = 0U;
        uint8_t btn = 0U;
        uint8_t hb = 0U;
        uint8_t u4 = 0U;
        uint8_t mux = 0U;
        uint8_t startup = 0U;
        if (frame.len >= 8U) {
            gear = static_cast<uint8_t>((frame.data[0] >> 4U) & 0x0FU);
            eco = static_cast<uint8_t>((frame.data[1] >> 4U) & 0x01U);
            carOnOff = static_cast<uint8_t>((frame.data[1] >> 5U) & 0x07U);
            btn = frame.data[2];
            hb = frame.data[3];
            u4 = frame.data[4];
            mux = frame.data[6];
            startup = frame.data[7];
        }

        file.printf("%u,0x%03lX,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
                    static_cast<unsigned>(frame.dtMs),
                    static_cast<unsigned long>(frame.id),
                    static_cast<unsigned long>(frame.id),
                    static_cast<unsigned>(frame.len),
                    static_cast<unsigned>(frame.ext),
                    static_cast<unsigned>(gear),
                    static_cast<unsigned>(carOnOff),
                    static_cast<unsigned>(eco),
                    static_cast<unsigned>(btn),
                    static_cast<unsigned>(hb),
                    static_cast<unsigned>(mux),
                    static_cast<unsigned>(u4),
                    static_cast<unsigned>(startup),
                    static_cast<unsigned>(frame.data[0]),
                    static_cast<unsigned>(frame.data[1]),
                    static_cast<unsigned>(frame.data[2]),
                    static_cast<unsigned>(frame.data[3]),
                    static_cast<unsigned>(frame.data[4]),
                    static_cast<unsigned>(frame.data[5]),
                    static_cast<unsigned>(frame.data[6]),
                    static_cast<unsigned>(frame.data[7]));
    }

    file.close();
    s_startupSniffDumped = true;
    Serial.printf("[STARTUP-SNIFF] saved %u frames to %s\n",
                  static_cast<unsigned>(s_startupSniffCount),
                  METASENSE_STARTUP_SNIFF_SAVE_PATH);
    return true;
#else
    return false;
#endif
}

bool printStartupSniffCapture()
{
#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
    if (!s_startupSniffDone) {
        return false;
    }
    emitStartupSniffCaptureManual();
    s_startupSniffDumped = true;
    return true;
#else
    return false;
#endif
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
