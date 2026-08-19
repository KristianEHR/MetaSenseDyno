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

#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
struct StartupSniffFrame {
    uint16_t dtMs = 0U;
    uint32_t id = 0U;
    uint8_t len = 0U;
    uint8_t ext = 0U;
    uint8_t data[8] = {0U};
};

static StartupSniffFrame s_startupSniffFrames[METASENSE_STARTUP_SNIFF_CAPTURE_MAX_FRAMES];
static uint16_t s_startupSniffCount = 0U;
static uint16_t s_startupSniffDropped = 0U;
static bool s_startupSniffActive = false;
static bool s_startupSniffDone = false;
static bool s_startupSniffDumped = false;
static uint32_t s_startupSniffStartMs = 0U;
static uint32_t s_startupSniffCompletedMs = 0U;
static uint32_t s_startupSniffArmAtMs = 0U;
static bool s_startupSniffArmInitialized = false;
static uint32_t s_startupSniffLast11aMs = 0U;

static bool isStartupSniffTrackedId(uint32_t id)
{
    return id == 0x11AU || id == 0x1D4U;
}

static void resetStartupSniffCapture()
{
    memset(s_startupSniffFrames, 0, sizeof(s_startupSniffFrames));
    s_startupSniffCount = 0U;
    s_startupSniffDropped = 0U;
    s_startupSniffActive = false;
    s_startupSniffDone = false;
    s_startupSniffDumped = false;
    s_startupSniffStartMs = 0U;
    s_startupSniffCompletedMs = 0U;
    s_startupSniffArmAtMs = 0U;
    s_startupSniffArmInitialized = false;
    s_startupSniffLast11aMs = 0U;
}

static void startStartupSniffCapture(uint32_t nowMs)
{
    if (s_startupSniffDone || s_startupSniffActive) {
        return;
    }
    s_startupSniffActive = true;
    s_startupSniffStartMs = nowMs;
    Serial.printf("[STARTUP-SNIFF] capture started (%lums)\n",
                  static_cast<unsigned long>(METASENSE_STARTUP_SNIFF_CAPTURE_MS));
}

static void recordStartupSniffFrame(uint32_t nowMs,
                                    uint32_t id,
                                    const uint8_t* data,
                                    uint8_t len,
                                    bool isExtended)
{
    if (s_startupSniffDone || data == nullptr) {
        return;
    }

    if (METASENSE_STARTUP_SNIFF_REARM_ON_11A_GAP_MS > 0U &&
        s_startupSniffDone &&
        !s_startupSniffActive &&
        isStartupSniffTrackedId(id)) {
        const bool gapExpired = (s_startupSniffLast11aMs == 0U) ||
                                ((nowMs - s_startupSniffLast11aMs) >= METASENSE_STARTUP_SNIFF_REARM_ON_11A_GAP_MS);
        if (gapExpired) {
            memset(s_startupSniffFrames, 0, sizeof(s_startupSniffFrames));
            s_startupSniffCount = 0U;
            s_startupSniffDropped = 0U;
            s_startupSniffDone = false;
            s_startupSniffDumped = false;
            s_startupSniffStartMs = 0U;
            s_startupSniffCompletedMs = 0U;
            Serial.printf("[STARTUP-SNIFF] re-armed after 11A gap >= %lums\n",
                          static_cast<unsigned long>(METASENSE_STARTUP_SNIFF_REARM_ON_11A_GAP_MS));
        }
    }

    if (!s_startupSniffActive && METASENSE_STARTUP_SNIFF_START_ON_FIRST_11A != 0U) {
        if (!s_startupSniffArmInitialized || nowMs < s_startupSniffArmAtMs) {
            if (isStartupSniffTrackedId(id)) {
                s_startupSniffLast11aMs = nowMs;
            }
            return;
        }
        if (isStartupSniffTrackedId(id)) {
            startStartupSniffCapture(nowMs);
        }
    }

    if (!s_startupSniffActive) {
        if (isStartupSniffTrackedId(id)) {
            s_startupSniffLast11aMs = nowMs;
        }
        return;
    }
    if (!isStartupSniffTrackedId(id)) {
        return;
    }

    if (s_startupSniffCount >= METASENSE_STARTUP_SNIFF_CAPTURE_MAX_FRAMES) {
        ++s_startupSniffDropped;
        return;
    }

    StartupSniffFrame& frame = s_startupSniffFrames[s_startupSniffCount++];
    const uint32_t dt = (nowMs >= s_startupSniffStartMs) ? (nowMs - s_startupSniffStartMs) : 0U;
    frame.dtMs = static_cast<uint16_t>((dt > 65535U) ? 65535U : dt);
    frame.id = id;
    frame.len = (len <= 8U) ? len : 8U;
    frame.ext = isExtended ? 1U : 0U;
    memset(frame.data, 0, sizeof(frame.data));
    memcpy(frame.data, data, frame.len);
    s_startupSniffLast11aMs = nowMs;
}

static void dumpStartupSniffCapture(uint32_t nowMs)
{
    if (s_startupSniffActive && !s_startupSniffDone) {
        const uint32_t elapsedMs = (nowMs >= s_startupSniffStartMs) ? (nowMs - s_startupSniffStartMs) : 0U;
        if (elapsedMs < METASENSE_STARTUP_SNIFF_CAPTURE_MS) {
            return;
        }

        s_startupSniffActive = false;
        s_startupSniffDone = true;
        s_startupSniffCompletedMs = nowMs;
        Serial.println("[STARTUP-SNIFF] capture complete");
    }
}

static void emitStartupSniffCaptureManual()
{
    const uint32_t elapsedMs = (s_startupSniffCompletedMs >= s_startupSniffStartMs)
                                   ? (s_startupSniffCompletedMs - s_startupSniffStartMs)
                                   : 0U;

    Serial.println("[STARTUP-SNIFF] manual dump");

    Serial.println("[STARTUP-SNIFF-BEGIN]");
    Serial.printf("meta,capture_ms,%lu,count,%u,dropped,%u\n",
                  static_cast<unsigned long>(elapsedMs),
                  static_cast<unsigned>(s_startupSniffCount),
                  static_cast<unsigned>(s_startupSniffDropped));

    // CSV columns mirror the CAN monitor 0x11A decode fields, with raw bytes retained for offline analysis.
    Serial.println("dt_ms,id_hex,id_dec,len,ext,gear,car_onoff,eco,btn,hb,mux,u4,startup,b0,b1,b2,b3,b4,b5,b6,b7");

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
            gear = static_cast<uint8_t>((frame.data[0] >> 4U) & 0x0FU);      // 4|4@1+
            eco = static_cast<uint8_t>((frame.data[1] >> 4U) & 0x01U);       // 12|1@1+
            carOnOff = static_cast<uint8_t>((frame.data[1] >> 5U) & 0x07U);  // 13|3@1+
            btn = frame.data[2];                                              // 16|8@1+
            hb = frame.data[3];                                               // 24|8@1+
            u4 = frame.data[4];                                               // 32|8@1+ (muxed)
            mux = frame.data[6];                                              // 48|8@1+
            startup = frame.data[7];                                          // 56|8@1+ (muxed)
        }

        Serial.printf("%u,0x%03lX,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
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

    Serial.println("[STARTUP-SNIFF-END]");
}
#endif

#if METASENSE_CAN_ID_SCAN
struct SniffIdEntry {
    uint32_t id = 0xFFFFFFFFU;
    uint8_t len = 0U;
    uint8_t data[8] = {0U};
    uint32_t frames = 0U;
    uint32_t lastMs = 0U;
    uint32_t lastReportedFrames = 0U;
    bool seen = false;
};

constexpr size_t kSniffIdSlots = 64U;
SniffIdEntry s_sniffIdEntries[kSniffIdSlots];

static void clearSniffIdEntries()
{
    memset(s_sniffIdEntries, 0, sizeof(s_sniffIdEntries));
}

static bool noteSniffId(uint32_t id, const uint8_t* data, uint8_t len)
{
    if (data == nullptr) {
        return false;
    }

    const uint8_t copyLen = (len <= 8U) ? len : 8U;

    for (size_t i = 0U; i < kSniffIdSlots; ++i) {
        if (s_sniffIdEntries[i].seen && s_sniffIdEntries[i].id == id) {
            ++s_sniffIdEntries[i].frames;
            s_sniffIdEntries[i].lastMs = millis();
            s_sniffIdEntries[i].len = copyLen;
            memset(s_sniffIdEntries[i].data, 0, sizeof(s_sniffIdEntries[i].data));
            memcpy(s_sniffIdEntries[i].data, data, copyLen);
            return false;
        }
        if (!s_sniffIdEntries[i].seen) {
            s_sniffIdEntries[i].id = id;
            s_sniffIdEntries[i].len = copyLen;
            s_sniffIdEntries[i].seen = true;
            s_sniffIdEntries[i].frames = 1U;
            s_sniffIdEntries[i].lastMs = millis();
            s_sniffIdEntries[i].lastReportedFrames = 0U;
            memset(s_sniffIdEntries[i].data, 0, sizeof(s_sniffIdEntries[i].data));
            memcpy(s_sniffIdEntries[i].data, data, copyLen);
            return true;
        }
    }
    return false;
}

static bool isSuppressedIdentifierId(uint32_t id)
{
    return id == 0x120U || id == 0x1DCU || id == 0x55BU || id == 0x50BU || id == 0x05BU;
}

static void printFrameIdentifierReport(uint32_t nowMs)
{
    static uint32_t lastReportMs = 0U;
    const uint32_t REPORT_PERIOD_MS = 2000U;
    if (lastReportMs != 0U && (nowMs - lastReportMs) < REPORT_PERIOD_MS) {
        return;
    }
    const uint32_t dtMs = (lastReportMs == 0U) ? REPORT_PERIOD_MS : (nowMs - lastReportMs);
    lastReportMs = nowMs;

    uint32_t active = 0U;
    for (size_t i = 0U; i < kSniffIdSlots; ++i) {
        if (!s_sniffIdEntries[i].seen || isSuppressedIdentifierId(s_sniffIdEntries[i].id)) {
            continue;
        }
        const uint32_t deltaFrames = s_sniffIdEntries[i].frames - s_sniffIdEntries[i].lastReportedFrames;
        if (deltaFrames == 0U) {
            continue;
        }
        ++active;
    }

    Serial.printf("[CAN-ID-REPORT] active=%lu dt=%lums\n",
                  static_cast<unsigned long>(active),
                  static_cast<unsigned long>(dtMs));

    for (size_t i = 0U; i < kSniffIdSlots; ++i) {
        if (!s_sniffIdEntries[i].seen || isSuppressedIdentifierId(s_sniffIdEntries[i].id)) {
            continue;
        }
        const uint32_t deltaFrames = s_sniffIdEntries[i].frames - s_sniffIdEntries[i].lastReportedFrames;
        if (deltaFrames == 0U) {
            continue;
        }
        const float hz = (dtMs > 0U)
            ? (1000.0f * static_cast<float>(deltaFrames) / static_cast<float>(dtMs))
            : 0.0f;
        const uint32_t ageMs = (s_sniffIdEntries[i].lastMs == 0U) ? 0U : (nowMs - s_sniffIdEntries[i].lastMs);

        Serial.printf("[CAN-ID] id=0x%03lX d=%lu hz=%.1f n=%lu age=%lums len=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                      static_cast<unsigned long>(s_sniffIdEntries[i].id),
                      static_cast<unsigned long>(deltaFrames),
                      hz,
                      static_cast<unsigned long>(s_sniffIdEntries[i].frames),
                      static_cast<unsigned long>(ageMs),
                      static_cast<unsigned>(s_sniffIdEntries[i].len),
                      static_cast<unsigned>(s_sniffIdEntries[i].data[0]),
                      static_cast<unsigned>(s_sniffIdEntries[i].data[1]),
                      static_cast<unsigned>(s_sniffIdEntries[i].data[2]),
                      static_cast<unsigned>(s_sniffIdEntries[i].data[3]),
                      static_cast<unsigned>(s_sniffIdEntries[i].data[4]),
                      static_cast<unsigned>(s_sniffIdEntries[i].data[5]),
                      static_cast<unsigned>(s_sniffIdEntries[i].data[6]),
                      static_cast<unsigned>(s_sniffIdEntries[i].data[7]));

        s_sniffIdEntries[i].lastReportedFrames = s_sniffIdEntries[i].frames;
    }
}

static size_t countSniffIds()
{
    size_t count = 0U;
    for (size_t i = 0U; i < kSniffIdSlots; ++i) {
        if (s_sniffIdEntries[i].seen) {
            ++count;
        }
    }
    return count;
}

static void printSniffIdSnapshot()
{
    const size_t count = countSniffIds();
    Serial.printf("CAN-SNIFF-SNAPSHOT total=%u\n", static_cast<unsigned>(count));
    for (size_t i = 0U; i < kSniffIdSlots; ++i) {
        if (!s_sniffIdEntries[i].seen) {
            continue;
        }
        Serial.printf("0x%03lX ", static_cast<unsigned long>(s_sniffIdEntries[i].id));
        for (uint8_t b = 0U; b < s_sniffIdEntries[i].len; ++b) {
            Serial.printf("%02X", static_cast<unsigned>(s_sniffIdEntries[i].data[b]));
            if (b + 1U < s_sniffIdEntries[i].len) {
                Serial.print(' ');
            }
        }
        Serial.println();
    }
}

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

bool isRxIdExcluded(uint32_t id)
{
    // Closed RX policy: explicitly ignore command/echo families.
    if (id == 0x120U || id == 0x11AU) {
        return true;
    }
#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
    return false;
#else
    return id == 0x1D4U;
#endif
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
    if (id != 0x50BU && id != 0x11AU) {
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
            // 0x1DA is CRC-gated; only decode if CRC is good
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
