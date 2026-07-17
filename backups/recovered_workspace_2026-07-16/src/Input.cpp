#include "Input.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <freertos/semphr.h>
#include <math.h>
#include <esp_timer.h>
#include <Wire.h>

#include <Adafruit_BME680.h>
#include <Adafruit_NAU7802.h>

#include "controlTask.h"
#include "I2cBusLock.h"
#include "DynoStateMachine.h"
#include "HardwareOutputStateMachine.h"
#include "Settings.h"
#include "RunStorage.h"
#include "TempHAL.h"
#include "Inverter.h"
#include "CANBus.h"
#include "globals.h"

namespace MetaSense::WebSocketServer {

AsyncWebSocket& socket();

} // namespace MetaSense::WebSocketServer


namespace { // LOCAL SCOPE

static MetaSense::Telemetry tele;

// filter state
static float rpmFilt     = 0.0f;
static float drumRpmFilt = 0.0f;
static float loadKgFilt  = 0.0f;
static float torqueFilt  = 0.0f;
static float filteredAdc = 0.0f;
constexpr uint8_t kLoadRawAverageWindowMax = 255;
constexpr uint8_t kLoadRawAverageWindowDefault = 5;
constexpr uint8_t kLoadFilterModeMovingAverage = 0;
constexpr uint8_t kLoadFilterModeTwoStageMovingAverage = 1;
static float loadRawAverageBuffer[kLoadRawAverageWindowMax] = {0.0f};
static uint8_t loadRawAverageCount = 0;
static uint8_t loadRawAverageIndex = 0;
static float loadRawAverageSum = 0.0f;
static uint8_t loadRawAverageActiveWindow = kLoadRawAverageWindowDefault;
static float loadKgAverageBuffer[kLoadRawAverageWindowMax] = {0.0f};
static uint8_t loadKgAverageCount = 0;
static uint8_t loadKgAverageIndex = 0;
static float loadKgAverageSum = 0.0f;
static uint8_t loadKgAverageActiveWindow = kLoadRawAverageWindowDefault;
constexpr uint8_t kTachoRawAverageWindow = kLoadRawAverageWindowDefault;
static float tachoRawAverageBuffer[kTachoRawAverageWindow] = {0.0f};
static uint8_t tachoRawAverageCount = 0;
static uint8_t tachoRawAverageIndex = 0;
static float tachoRawAverageSum = 0.0f;
constexpr uint8_t kAuxRawAverageWindow = 8;
static float massflowRawAverageBuffer[kAuxRawAverageWindow] = {0.0f};
static uint8_t massflowRawAverageCount = 0;
static uint8_t massflowRawAverageIndex = 0;
static float massflowRawAverageSum = 0.0f;
static float lambdaRawAverageBuffer[kAuxRawAverageWindow] = {0.0f};
static uint8_t lambdaRawAverageCount = 0;
static uint8_t lambdaRawAverageIndex = 0;
static float lambdaRawAverageSum = 0.0f;

// RPM inputs
static float canRpm   = 0.0f;
static float tachoRpm = 0.0f;
static float tachoCal = 10.0f;   // tachogen calibration factor
constexpr float kLoadCellRawScale = 0.01f;
static float zeroOffset = 0.0f;
static float zeroDeadbandRaw = 0.0f;
static float calibrationFactor = 0.01f;
static TempHAL egtDigital;
static bool egtDigitalReady = false;
static String i2cScanSummary = "";
static Adafruit_BME680 ambientBme;
static Adafruit_NAU7802 loadCellNau;
static SemaphoreHandle_t loadCellMutex = nullptr;
static TaskHandle_t loadCellSamplerTaskHandle = nullptr;
static TaskHandle_t calibrationTaskHandle = nullptr;
static bool ambientBmeReady = false;
static bool ambientBmeReadPending = false;
static uint32_t ambientBmeReadReadyMs = 0;
static bool loadCellNauReady = false;
static bool loadCellNauLdoConfigured = false;
static bool loadCellNauInternalCalOk = false;
static uint8_t loadCellNauInternalCalAttempts = 0;
static uint16_t loadCellCurrentRateSps = 0;
static uint8_t loadCellCurrentGainValue = 128;
static uint32_t lastLoadCellNauRetryMs = 0;
#if defined(METASENSE_STREAM_DIAGNOSTICS) && (METASENSE_STREAM_DIAGNOSTICS != 0)
static uint32_t wsSentTotal = 0;
static uint32_t wsSkipBacklogTotal = 0;
static uint32_t wsSkipFullTotal = 0;
static uint32_t wsSkipNoSendTotal = 0;
#endif
#if defined(METASENSE_JSON_DELAY_COUNTERS) && (METASENSE_JSON_DELAY_COUNTERS != 0)
static uint16_t wsJsonLastLen = 0;
static uint16_t wsJsonMaxLen = 0;
static uint32_t wsJsonOverReserveTotal = 0;
#endif
static float ambientTempC = 20.0f;
static float ambientHumidityPct = 50.0f;
static float ambientPressureHpa = 1013.25f;
static uint32_t lastAmbientSampleMs = 0;

// CAN RPM validity timeout + plausibility
static uint32_t lastCanRpmUpdate   = 0;
static const uint32_t CAN_RPM_TIMEOUT_MS = 100;
#ifndef METASENSE_CAN_RPM_MIN_UPDATE_MS
#define METASENSE_CAN_RPM_MIN_UPDATE_MS 10
#endif
static const uint32_t CAN_RPM_MIN_UPDATE_MS = METASENSE_CAN_RPM_MIN_UPDATE_MS;
static float lastCanRpm            = 0.0f;
static const float CAN_MAX_JUMP    = 2000.0f;
static float leafCanRpmMonitor = 0.0f;
static uint32_t lastCanRpmMonitorUpdate = 0;
static float canInvTempC = 0.0f;
static float canStatorTempC = 0.0f;
static float canCoolantTempC = 0.0f;
static bool canInvReady = false;
static bool canInvFault = false;
static bool canInvWarning = false;
static bool canInvLimp = false;
static uint32_t lastCanTempUpdate = 0;
static uint32_t lastCanStatusUpdate = 0;
static uint32_t lastCanLeafAnyUpdate = 0;
static uint32_t lastCanRpmFrameMs = 0;
static uint32_t lastCanTorqueFrameMs = 0;
static uint32_t lastCanTempsFrameMs = 0;
static uint32_t lastCanStatusFrameMs = 0;
static bool vcuDebugSimMode = false;
static bool vcuDebugInv12v = false;
static float vcuDebugHvVoltage = 0.0f;
static float vcuDebugTorqueDemandNm = 0.0f;
static float s_leafLastSentTorqueNm = 0.0f;
static uint32_t s_leafLastSentTorqueMs = 0;
static uint32_t s_leafTorqueTrackStartMs = 0;
static uint16_t s_leafTorqueTrackLatencyMs = 0;
static bool s_leafTorqueTrackPending = false;
static float s_leafTorqueTrackTargetNm = 0.0f;
static bool vcuDebugRPlus = false;
static bool vcuDebugPrecharge = false;
static bool vcuDebugSsr = false;
static bool vcuDebugRMinus = false;
static uint32_t lastLeafTxMs = 0;
static bool lastCanDiagInitialized = false;
static bool lastCanDiagReady = false;
static uint8_t lastCanDiagState = 0xFF;
static uint32_t lastCanDiagTxFailures = 0;
static uint32_t lastCanDiagTxWhileNotReady = 0;
static uint32_t lastCanDiagRecoveries = 0;
static uint32_t lastCanDiagBusOff = 0;
static uint32_t lastCanDiagStatusQueryFailures = 0;
static uint32_t lastCanDiagTwaiRxQueued = 0;
static uint32_t lastCanDiagTwaiTxQueued = 0;
static uint32_t lastCanDiagTwaiRxMissed = 0;
static uint32_t lastCanDiagTwaiRxOverrun = 0;
static uint32_t lastCanDiagTwaiArbLost = 0;
static uint32_t lastCanDiagTwaiBusError = 0;
static uint32_t lastCanDiagTwaiTxErr = 0;
static uint32_t lastCanDiagTwaiRxErr = 0;
static uint32_t lastCanEventLogMs = 0;
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
static uint32_t lastCanAltDiagMs = 0;
static uint32_t lastCanAlt120FramesLogged = 0;
static uint32_t lastCanAlt55aFramesLogged = 0;
#endif
static uint32_t lastCanBusOffSeen = 0;
static uint32_t lastCanStatusQueryFailuresSeen = 0;

// RPM delta error
static bool  rpmDeltaError         = false;
static bool  canFallbackActive     = false;
static bool  activeRpmFromCan      = false;
static const float RPM_DELTA_LIMIT = 100.0f;
static const uint32_t CAN_TEMP_TIMEOUT_MS = 1000;
static const uint32_t CAN_TX_PERIOD_MS = 10;
static const uint32_t CAN_RX_CHECK_PERIOD_MS = 20;
static const uint32_t CAN_RX_TARGET_MAX_AGE_MS = 250;
static const uint32_t CAN_RX_MISSING_LOG_PERIOD_MS = 5000;
static const uint32_t CAN_EVENT_LOG_MIN_PERIOD_MS = 5000;
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
static const uint32_t CAN_ALT_DIAG_LOG_PERIOD_MS = 5000;
static const uint32_t CAN_PRE_DIAG_LOG_PERIOD_MS = 5000;
#endif

#ifndef METASENSE_LEAF_VCM_CHECKLIST_MODE
#define METASENSE_LEAF_VCM_CHECKLIST_MODE 1
#endif
#ifndef METASENSE_LEAF_VCM_DIAGNOSTICS
#define METASENSE_LEAF_VCM_DIAGNOSTICS 0
#endif
#ifndef METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS
#define METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS 0
#endif
#ifndef METASENSE_TEST_TORQUE_OVERRIDE_ENABLED
#define METASENSE_TEST_TORQUE_OVERRIDE_ENABLED 0
#endif
#ifndef METASENSE_TEST_TORQUE_OVERRIDE_NM
#define METASENSE_TEST_TORQUE_OVERRIDE_NM 25.0f
#endif
// Allow scheduler jitter from WiFi/web/FS tasks without tripping a false VCM fault.
static const uint32_t CAN_TX_MAX_GAP_MS = 120;

enum class LeafVcmBringupState : uint8_t {
    CanOnline = 0,
    Precharge = 1,
    HvOk = 2,
    Ready = 3,
    Fault = 4
};

const char* leafVcmStateName(LeafVcmBringupState state)
{
    switch (state) {
        case LeafVcmBringupState::CanOnline: return "CAN_ONLINE";
        case LeafVcmBringupState::Precharge: return "PRECHARGE";
        case LeafVcmBringupState::HvOk: return "HV_OK";
        case LeafVcmBringupState::Ready: return "READY";
        case LeafVcmBringupState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

static LeafVcmBringupState s_leafVcmState = LeafVcmBringupState::CanOnline;
static uint32_t s_leafVcmStateSinceMs = 0;
static bool s_leafTxGapFault = false;
static bool s_leafVcmFaultLatched = false;
static uint32_t s_leafVcmLastFaultMs = 0;
static bool s_leafCanPartnerSeen = false;
static uint32_t s_leafSimLastInjectMs = 0;
static bool s_leafSimLogPrinted = false;
static bool s_leafSimFeedbackActive = false;
static bool s_leafListenOnlyLogPrinted = false;
static bool s_leafHandshakeSent = false;
static bool s_leafHandshakeArmed = false;
static uint8_t s_leafHandshakeSentCount = 0;
static uint8_t s_leafHandshakeAttemptCount = 0;
static uint32_t s_leafHandshakeStartMs = 0;
static uint32_t s_leafHandshakeLastAttemptMs = 0;
static bool s_leafHandshakePromotedLogPrinted = false;
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
static uint32_t s_leafPreStatusLastMs = 0;
#endif
static uint32_t s_leafRxDiagLastMs = 0;
static uint32_t s_leafRxWarnLastMs = 0;
static uint32_t s_leafRxAwaitPartnerLastMs = 0;
static uint32_t s_leafTxGapTestCycleStartMs = 0;
static bool s_leafTxGapTestActive = false;
static bool s_leafTxGapTestLoggedStart = false;
static bool s_leafTxGapTestLoggedEnd = false;

constexpr uint32_t kLeafVcmCanOnlineDetectMs = 500;
constexpr uint32_t kLeafVcmFeedbackTimeoutMs = 250;
constexpr uint32_t kLeafVcmPrechargeTimeoutMs = 2500;
constexpr float kLeafVcmHvReadyVoltageV = 300.0f;
constexpr uint32_t kLeafVcmHvOkSettleMs = 100;
constexpr uint32_t kLeafVcmFaultRecoverMs = 500;
constexpr float kLeafTorqueTrackStepNm = 2.0f;
constexpr float kLeafTorqueTrackAbsTolNm = 3.0f;
constexpr float kLeafTorqueTrackRelTol = 0.10f;
#ifndef METASENSE_LEAF_TORQUE_TRACK_TIMEOUT_MS
#define METASENSE_LEAF_TORQUE_TRACK_TIMEOUT_MS 3000
#endif
constexpr uint32_t kLeafTorqueTrackTimeoutMs = METASENSE_LEAF_TORQUE_TRACK_TIMEOUT_MS;
constexpr uint32_t kLeafSimFeedbackPeriodMs = 20;
constexpr uint32_t kLeafSimBootDelayMs = 1000;
constexpr uint32_t kLeafSimReadyDelayMs = 1800;
constexpr uint32_t kLeafRxAwaitPartnerLogMs = 5000;
constexpr uint16_t kLeafHandshakeTargetSends = 80;
constexpr uint16_t kLeafHandshakeMaxAttempts = 200;
constexpr uint32_t kLeafHandshakeWindowMs = 5000;
constexpr uint32_t kLeafHandshakeAttemptPeriodMs = 20;

#ifndef METASENSE_LEAF_CAN_RX_ENABLED
#define METASENSE_LEAF_CAN_RX_ENABLED 1
#endif
#ifndef METASENSE_LEAF_CAN_TX_ENABLED
#define METASENSE_LEAF_CAN_TX_ENABLED 0
#endif
#ifndef METASENSE_LEAF_CAN_LISTEN_ONLY
#define METASENSE_LEAF_CAN_LISTEN_ONLY 0
#endif
#ifndef METASENSE_LEAF_CAN_HANDSHAKE_ON_FIRST_1DA
#define METASENSE_LEAF_CAN_HANDSHAKE_ON_FIRST_1DA 0
#endif
#ifndef METASENSE_LEAF_VARIANT_120_55A
#define METASENSE_LEAF_VARIANT_120_55A 0
#endif
#ifndef METASENSE_LEAF_CAN_VARIANT_READY_FALLBACK
#define METASENSE_LEAF_CAN_VARIANT_READY_FALLBACK 1
#endif
#ifndef METASENSE_LEAF_CAN_TX_PIN
#define METASENSE_LEAF_CAN_TX_PIN 4
#endif
#ifndef METASENSE_LEAF_CAN_RX_PIN
#define METASENSE_LEAF_CAN_RX_PIN 5
#endif
#ifndef METASENSE_LEAF_CAN_MAX_FRAMES_PER_LOOP
#define METASENSE_LEAF_CAN_MAX_FRAMES_PER_LOOP 8
#endif
#ifndef METASENSE_LEAF_TX_GAP_TEST_ENABLE
#define METASENSE_LEAF_TX_GAP_TEST_ENABLE 0
#endif
#ifndef METASENSE_LEAF_TX_GAP_TEST_PERIOD_MS
#define METASENSE_LEAF_TX_GAP_TEST_PERIOD_MS 15000
#endif
#ifndef METASENSE_LEAF_TX_GAP_TEST_DURATION_MS
#define METASENSE_LEAF_TX_GAP_TEST_DURATION_MS 500
#endif
#ifndef METASENSE_FORCE_TACHO_RPM_SOURCE
// Safety: keep control-loop RPM on tachogen to avoid CAN RPM dual-writer races.
#define METASENSE_FORCE_TACHO_RPM_SOURCE 1
#endif

constexpr bool kLeafCanHandshakeOnFirst1da = (METASENSE_LEAF_CAN_HANDSHAKE_ON_FIRST_1DA != 0);
// Keep fallback independent from legacy 0x120/0x55A decode path.
constexpr bool kLeafCanVariantReadyFallback = (METASENSE_LEAF_CAN_VARIANT_READY_FALLBACK != 0);
constexpr bool kLeafCanTxActive = (METASENSE_LEAF_CAN_TX_ENABLED != 0) &&
                                  (METASENSE_LEAF_CAN_LISTEN_ONLY == 0) &&
                                  !kLeafCanHandshakeOnFirst1da;

const MetaSense::CANBus::Config kLeafCanConfig = []() {
    MetaSense::CANBus::Config config;
    config.txPin = METASENSE_LEAF_CAN_TX_PIN;
    config.rxPin = METASENSE_LEAF_CAN_RX_PIN;
    config.maxFramesPerPoll = METASENSE_LEAF_CAN_MAX_FRAMES_PER_LOOP;
    config.initRetryMs = 2000U;
    return config;
}();

// safety limits
constexpr float RPM_MAX_LIMIT   = 18000.0f;
constexpr float EGT_MAX_LIMIT_C = 950.0f;

// PI controller
constexpr float TORQUE_MIN = -200.0f;
constexpr float TORQUE_MAX =  200.0f;
constexpr float RPM_SETPOINT_MAX = RPM_MAX_LIMIT;
#ifndef METASENSE_WS_FAST_PERIOD_MS
#define METASENSE_WS_FAST_PERIOD_MS 25
#endif
#ifndef METASENSE_WS_SLOW_PERIOD_MS
#define METASENSE_WS_SLOW_PERIOD_MS 500
#endif
constexpr uint32_t kWebSocketPublishPeriodMs = METASENSE_WS_FAST_PERIOD_MS;
constexpr uint32_t kWebSocketSlowPublishPeriodMs = METASENSE_WS_SLOW_PERIOD_MS;
constexpr uint8_t kWebSocketSlowTelemetrySlices = 15;
#ifndef METASENSE_TELEMETRY_PROFILE_DEFAULT_TREND
#define METASENSE_TELEMETRY_PROFILE_DEFAULT_TREND 0
#endif
#ifndef METASENSE_TREND_MINIMAL_TELEMETRY
// When enabled, trend.html receives only the fields needed for live trend plots.
#define METASENSE_TREND_MINIMAL_TELEMETRY 1
#endif
#ifndef METASENSE_DASHBOARD_TRANSITIONAL_RUN_FIELDS
// When enabled, low-rate dashboard run-condition fields are sent only on change.
#define METASENSE_DASHBOARD_TRANSITIONAL_RUN_FIELDS 1
#endif

enum class TelemetryProfile : uint8_t {
    Full = 0,
    TrendMinimal = 1
};

static TelemetryProfile gTelemetryProfile =
    METASENSE_TELEMETRY_PROFILE_DEFAULT_TREND ? TelemetryProfile::TrendMinimal : TelemetryProfile::Full;
static bool gUiModeHintTrend = false;

#ifndef METASENSE_STREAM_DIAGNOSTICS
#define METASENSE_STREAM_DIAGNOSTICS 0
#endif
// Compile-time facility for JSON delay counter diagnostics (disabled by default for production)
#ifndef METASENSE_JSON_DELAY_COUNTERS
#define METASENSE_JSON_DELAY_COUNTERS 0  // Set to 1 to enable JSON size measurements
#endif
constexpr float kRuntimeKpMin = 0.005f;
constexpr float kRuntimeKpMax = 0.200f;
constexpr float kRuntimeKpAlpha = 0.12f;
constexpr float kRuntimeKpApplyDelta = 0.001f;
#ifndef METASENSE_AMBIENT_SAMPLE_PERIOD_MS
#define METASENSE_AMBIENT_SAMPLE_PERIOD_MS 100
#endif
constexpr uint32_t kAmbientSamplePeriodMs = METASENSE_AMBIENT_SAMPLE_PERIOD_MS;
constexpr uint32_t kLoadCellNauRetryPeriodMs = 5000;
constexpr uint16_t kMcp9600BootSettleDelayMs = 1000;
constexpr uint16_t kMcp9600BootRetryDelayMs = 500;
constexpr bool kEgtOnlyI2cInitMode = false;
constexpr bool kNauOnlyI2cInitMode = false;
constexpr float kAuxAnalogFilterAlpha = 0.20f;
constexpr float kMassflowMaxM3h = 300.0f;
constexpr NAU7802_Gain kLoadCellDefaultRuntimeGain = NAU7802_GAIN_128;
constexpr NAU7802_Gain kLoadCellStableGain = NAU7802_GAIN_128;
constexpr NAU7802_SampleRate kLoadCellDefaultRuntimeRate = NAU7802_RATE_320SPS;
constexpr NAU7802_SampleRate kLoadCellStableRate = NAU7802_RATE_320SPS;
constexpr float kLambdaMin = 0.50f;
constexpr uint8_t kCalibrationSkipFirstSamples = 10;
constexpr uint32_t kCalibrationSettlingMs = 1000;
constexpr uint32_t kCalibrationDurationMs = 5000;
constexpr uint32_t kCalibrationSamplePeriodMs = 50;
constexpr uint8_t kLoadRawBurstSamples = 4;
constexpr uint8_t kLoadRawStatsMaxSamples = 96;
constexpr uint32_t kLoadCellSamplerPeriodMs = 5;
constexpr uint8_t kLoadCellSamplerWindow = 12;
constexpr float kLoadCellOutlierMinJumpRaw = 400.0f;
constexpr float kLoadCellOutlierRelativeJump = 0.25f;
constexpr float kTareDeadbandMaxRaw = 40.0f;
constexpr uint8_t kNauRateSwitchMinDiscardSamples = 4;
constexpr uint8_t kNauRateSwitchMaxDiscardSamples = 12;
constexpr uint32_t kNauRateSwitchMinSettleMs = 80;
constexpr uint32_t kNauRateSwitchMaxSettleMs = 900;
constexpr uint32_t kNauRateSwitchRestartDelayMs = 2;
constexpr uint16_t kNauLdoRampDelayMs = 250;
constexpr NAU7802_LDO kLoadCellDefaultLdo = NAU7802_LDO_3V0;
constexpr uint8_t kNauInitFlushReadings = 10;
constexpr uint8_t kNauInternalCalMaxAttempts = 3;
constexpr uint16_t kNauInternalCalRetryDelayMs = 1000;

static volatile bool calibRequestPending = false;
static volatile bool tareRequestPending = false;
static volatile bool calibBusy = false;
static volatile bool loadCellExclusiveSampling = false;
static volatile uint32_t loadCellSamplerLastUs = 0;
static volatile uint32_t loadCellSamplerMaxUs = 0;
static volatile uint32_t loadCellSamplerEmaUs = 0;
static volatile uint32_t loadCellSamplerLoops = 0;
static float calibKnownWeightKg = 0.0f;
static float loadCellSamplerBuffer[kLoadCellSamplerWindow] = {0.0f};
static uint8_t loadCellSamplerCount = 0;
static uint8_t loadCellSamplerIndex = 0;
static float loadCellSamplerSum = 0.0f;
static float loadCellSamplerAverageRaw = 0.0f;
static bool loadCellSamplerValid = false;
static volatile uint64_t lastRawCaptureAppendUs = 0;
portMUX_TYPE calibMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE loadCellSamplerMux = portMUX_INITIALIZER_UNLOCKED;
constexpr float kLambdaMax = 1.50f;
constexpr size_t kFallbackRunMaxPoints = 150;
constexpr uint32_t kFallbackRunSamplePeriodMs = 100;
static float fallbackRunRpm[kFallbackRunMaxPoints] = {0.0f};
static float fallbackRunTimeSec[kFallbackRunMaxPoints] = {0.0f};
static float fallbackRunKw[kFallbackRunMaxPoints] = {0.0f};
static float fallbackRunTorque[kFallbackRunMaxPoints] = {0.0f};
static size_t fallbackRunCount = 0;
static uint32_t fallbackRunStartMs = 0;
static uint32_t fallbackRunLastSampleMs = 0;

// helpers
float lpFilter(float prev, float input, float alpha)
{
    return prev + alpha * (input - prev);
}

uint32_t elapsedMsSafe(uint32_t now, uint32_t since)
{
    if (since == 0U || now < since) {
        return 0U;
    }
    return now - since;
}

void formatCanPayloadHex(const uint8_t* data, uint8_t len, char* out, size_t outSize)
{
    if (outSize == 0U) {
        return;
    }
    out[0] = '\0';
    if (data == nullptr || len == 0U) {
        return;
    }

    size_t used = 0U;
    for (uint8_t i = 0; i < len; ++i) {
        const int written = snprintf(out + used,
                                     outSize - used,
                                     (i == 0U) ? "%02X" : " %02X",
                                     static_cast<unsigned>(data[i]));
        if (written <= 0) {
            break;
        }
        const size_t advanced = static_cast<size_t>(written);
        if (advanced >= (outSize - used)) {
            used = outSize - 1U;
            break;
        }
        used += advanced;
    }
}

uint16_t nauSampleRateToSps(NAU7802_SampleRate rate)
{
    switch (rate) {
        case NAU7802_RATE_10SPS:  return 10;
        case NAU7802_RATE_20SPS:  return 20;
        case NAU7802_RATE_40SPS:  return 40;
        case NAU7802_RATE_80SPS:  return 80;
        case NAU7802_RATE_320SPS: return 320;
        default: return 0;
    }
}

uint8_t nauGainToValue(NAU7802_Gain gain)
{
    switch (gain) {
        case NAU7802_GAIN_1: return 1;
        case NAU7802_GAIN_2: return 2;
        case NAU7802_GAIN_4: return 4;
        case NAU7802_GAIN_8: return 8;
        case NAU7802_GAIN_16: return 16;
        case NAU7802_GAIN_32: return 32;
        case NAU7802_GAIN_64: return 64;
        case NAU7802_GAIN_128: return 128;
        default: return 0;
    }
}

NAU7802_Gain loadCellGainFromSetting(uint16_t gainValue)
{
    switch (gainValue) {
        case 1: return NAU7802_GAIN_1;
        case 2: return NAU7802_GAIN_2;
        case 4: return NAU7802_GAIN_4;
        case 8: return NAU7802_GAIN_8;
        case 16: return NAU7802_GAIN_16;
        case 32: return NAU7802_GAIN_32;
        case 64: return NAU7802_GAIN_64;
        case 128: return NAU7802_GAIN_128;
        default: return kLoadCellDefaultRuntimeGain;
    }
}

NAU7802_SampleRate loadCellRateFromSetting(uint16_t rateSps)
{
    switch (rateSps) {
        case 10: return NAU7802_RATE_10SPS;
        case 20: return NAU7802_RATE_20SPS;
        case 40: return NAU7802_RATE_40SPS;
        case 80: return NAU7802_RATE_80SPS;
        case 320: return NAU7802_RATE_320SPS;
        default: return kLoadCellDefaultRuntimeRate;
    }
}

NAU7802_Gain currentRuntimeLoadCellGain()
{
    return loadCellGainFromSetting(MetaSense::Settings::loadCellGain);
}

NAU7802_SampleRate currentRuntimeLoadCellRate()
{
    return loadCellRateFromSetting(MetaSense::Settings::loadCellRateSps);
}

struct NauTransitionProfile {
    NAU7802_Gain gain;
    NAU7802_SampleRate rate;
    uint32_t settleMs;
    uint8_t discardSamples;
};

#define NAU_PROFILE(g, r, s, d) { g, r, s, d }
constexpr NauTransitionProfile kNauTransitionProfiles[] = {
    NAU_PROFILE(NAU7802_GAIN_1,   NAU7802_RATE_10SPS,  700, 7),
    NAU_PROFILE(NAU7802_GAIN_1,   NAU7802_RATE_20SPS,  450, 9),
    NAU_PROFILE(NAU7802_GAIN_1,   NAU7802_RATE_40SPS,  260, 10),
    NAU_PROFILE(NAU7802_GAIN_1,   NAU7802_RATE_80SPS,  160, 10),
    NAU_PROFILE(NAU7802_GAIN_1,   NAU7802_RATE_320SPS, 100, 12),
    NAU_PROFILE(NAU7802_GAIN_2,   NAU7802_RATE_10SPS,  700, 7),
    NAU_PROFILE(NAU7802_GAIN_2,   NAU7802_RATE_20SPS,  450, 9),
    NAU_PROFILE(NAU7802_GAIN_2,   NAU7802_RATE_40SPS,  260, 10),
    NAU_PROFILE(NAU7802_GAIN_2,   NAU7802_RATE_80SPS,  160, 10),
    NAU_PROFILE(NAU7802_GAIN_2,   NAU7802_RATE_320SPS, 100, 12),
    NAU_PROFILE(NAU7802_GAIN_4,   NAU7802_RATE_10SPS,  700, 7),
    NAU_PROFILE(NAU7802_GAIN_4,   NAU7802_RATE_20SPS,  450, 9),
    NAU_PROFILE(NAU7802_GAIN_4,   NAU7802_RATE_40SPS,  260, 10),
    NAU_PROFILE(NAU7802_GAIN_4,   NAU7802_RATE_80SPS,  160, 10),
    NAU_PROFILE(NAU7802_GAIN_4,   NAU7802_RATE_320SPS, 100, 12),
    NAU_PROFILE(NAU7802_GAIN_8,   NAU7802_RATE_10SPS,  700, 7),
    NAU_PROFILE(NAU7802_GAIN_8,   NAU7802_RATE_20SPS,  450, 9),
    NAU_PROFILE(NAU7802_GAIN_8,   NAU7802_RATE_40SPS,  260, 10),
    NAU_PROFILE(NAU7802_GAIN_8,   NAU7802_RATE_80SPS,  160, 10),
    NAU_PROFILE(NAU7802_GAIN_8,   NAU7802_RATE_320SPS, 100, 12),
    NAU_PROFILE(NAU7802_GAIN_16,  NAU7802_RATE_10SPS,  700, 7),
    NAU_PROFILE(NAU7802_GAIN_16,  NAU7802_RATE_20SPS,  450, 9),
    NAU_PROFILE(NAU7802_GAIN_16,  NAU7802_RATE_40SPS,  260, 10),
    NAU_PROFILE(NAU7802_GAIN_16,  NAU7802_RATE_80SPS,  160, 10),
    NAU_PROFILE(NAU7802_GAIN_16,  NAU7802_RATE_320SPS, 100, 12),
    NAU_PROFILE(NAU7802_GAIN_32,  NAU7802_RATE_10SPS,  700, 7),
    NAU_PROFILE(NAU7802_GAIN_32,  NAU7802_RATE_20SPS,  450, 9),
    NAU_PROFILE(NAU7802_GAIN_32,  NAU7802_RATE_40SPS,  260, 10),
    NAU_PROFILE(NAU7802_GAIN_32,  NAU7802_RATE_80SPS,  160, 10),
    NAU_PROFILE(NAU7802_GAIN_32,  NAU7802_RATE_320SPS, 100, 12),
    NAU_PROFILE(NAU7802_GAIN_64,  NAU7802_RATE_10SPS,  700, 7),
    NAU_PROFILE(NAU7802_GAIN_64,  NAU7802_RATE_20SPS,  450, 9),
    NAU_PROFILE(NAU7802_GAIN_64,  NAU7802_RATE_40SPS,  260, 10),
    NAU_PROFILE(NAU7802_GAIN_64,  NAU7802_RATE_80SPS,  160, 10),
    NAU_PROFILE(NAU7802_GAIN_64,  NAU7802_RATE_320SPS, 100, 12),
    NAU_PROFILE(NAU7802_GAIN_128, NAU7802_RATE_10SPS,  700, 7),
    NAU_PROFILE(NAU7802_GAIN_128, NAU7802_RATE_20SPS,  450, 9),
    NAU_PROFILE(NAU7802_GAIN_128, NAU7802_RATE_40SPS,  260, 10),
    NAU_PROFILE(NAU7802_GAIN_128, NAU7802_RATE_80SPS,  160, 10),
    NAU_PROFILE(NAU7802_GAIN_128, NAU7802_RATE_320SPS, 100, 12),
};
#undef NAU_PROFILE

const NauTransitionProfile* findNauTransitionProfile(NAU7802_Gain gain, NAU7802_SampleRate rate)
{
    for (const NauTransitionProfile& profile : kNauTransitionProfiles) {
        if (profile.gain == gain && profile.rate == rate) {
            return &profile;
        }
    }
    return nullptr;
}

void resetLoadCellSampler(float seed)
{
    portENTER_CRITICAL(&loadCellSamplerMux);
    loadCellSamplerCount = 0;
    loadCellSamplerIndex = 0;
    loadCellSamplerSum = 0.0f;
    loadCellSamplerAverageRaw = seed;
    const bool seedLooksReal = isfinite(seed) && (fabsf(seed) > kLoadCellOutlierMinJumpRaw);
    loadCellSamplerValid = seedLooksReal;

    for (uint8_t i = 0; i < kLoadCellSamplerWindow; ++i) {
        loadCellSamplerBuffer[i] = 0.0f;
    }

    if (seedLooksReal) {
        loadCellSamplerBuffer[0] = seed;
        loadCellSamplerSum = seed;
        loadCellSamplerCount = 1;
        loadCellSamplerIndex = 1;
    }
    portEXIT_CRITICAL(&loadCellSamplerMux);
}

void pushLoadCellSamplerRaw(float raw)
{
    if (!isfinite(raw)) {
        return;
    }

    // Suppress obvious conversion glitches before they contaminate the rolling
    // baseline, but do not block legitimate torque/load step changes.
    if (loadCellSamplerValid && loadCellSamplerCount >= 8) {
        portENTER_CRITICAL(&loadCellSamplerMux);
        const float baseline = loadCellSamplerAverageRaw;
        const uint8_t count = loadCellSamplerCount;
        portEXIT_CRITICAL(&loadCellSamplerMux);
        const float allowedJump = max(kLoadCellOutlierMinJumpRaw,
                                      fabsf(baseline) * kLoadCellOutlierRelativeJump);
        const float jump = fabsf(raw - baseline);
        const bool suspiciousZero = (raw == 0.0f && fabsf(baseline) > 100.0f);
        const bool notEnoughHistory = count < 8;
        if (!notEnoughHistory && suspiciousZero) {
            return;
        }

        if (!notEnoughHistory && jump > allowedJump) {
            const float direction = (raw >= baseline) ? 1.0f : -1.0f;
            raw = baseline + direction * allowedJump;
        }
    }

    portENTER_CRITICAL(&loadCellSamplerMux);
    if (loadCellSamplerCount < kLoadCellSamplerWindow) {
        loadCellSamplerBuffer[loadCellSamplerIndex] = raw;
        loadCellSamplerSum += raw;
        ++loadCellSamplerCount;
    } else {
        loadCellSamplerSum -= loadCellSamplerBuffer[loadCellSamplerIndex];
        loadCellSamplerBuffer[loadCellSamplerIndex] = raw;
        loadCellSamplerSum += raw;
    }

    ++loadCellSamplerIndex;
    if (loadCellSamplerIndex >= kLoadCellSamplerWindow) {
        loadCellSamplerIndex = 0;
    }

    loadCellSamplerAverageRaw = loadCellSamplerSum / static_cast<float>(loadCellSamplerCount);
    loadCellSamplerValid = true;
    portEXIT_CRITICAL(&loadCellSamplerMux);
}

float getLoadCellSamplerAverageRaw()
{
    portENTER_CRITICAL(&loadCellSamplerMux);
    const bool valid = loadCellSamplerValid;
    const float avg = loadCellSamplerAverageRaw;
    portEXIT_CRITICAL(&loadCellSamplerMux);

    return valid ? avg : filteredAdc;
}

uint8_t getConfiguredLoadRawAverageWindow()
{
    const float configured = MetaSense::Settings::loadAvgN;
    if (!isfinite(configured)) {
        return kLoadRawAverageWindowDefault;
    }

    const long rounded = lroundf(configured);
    if (rounded < 1) {
        return 1;
    }
    if (rounded > kLoadRawAverageWindowMax) {
        return kLoadRawAverageWindowMax;
    }
    return static_cast<uint8_t>(rounded);
}

uint8_t getConfiguredLoadKgAverageWindow()
{
    const float configured = MetaSense::Settings::loadAvgN2;
    if (!isfinite(configured)) {
        return kLoadRawAverageWindowDefault;
    }

    const long rounded = lroundf(configured);
    if (rounded < 1) {
        return 1;
    }
    if (rounded > kLoadRawAverageWindowMax) {
        return kLoadRawAverageWindowMax;
    }
    return static_cast<uint8_t>(rounded);
}

void resetLoadRawAverage(float seed)
{
    const uint8_t activeWindow = getConfiguredLoadRawAverageWindow();
    loadRawAverageActiveWindow = activeWindow;
    loadRawAverageCount = 0;
    loadRawAverageIndex = 0;
    loadRawAverageSum = 0.0f;

    for (uint8_t i = 0; i < kLoadRawAverageWindowMax; ++i) {
        loadRawAverageBuffer[i] = 0.0f;
    }

    if (isfinite(seed)) {
        loadRawAverageBuffer[0] = seed;
        loadRawAverageSum = seed;
        loadRawAverageCount = 1;
        loadRawAverageIndex = (activeWindow > 1) ? 1 : 0;
    }
}

void resetLoadKgAverage(float seed)
{
    const uint8_t activeWindow = getConfiguredLoadKgAverageWindow();
    loadKgAverageActiveWindow = activeWindow;
    loadKgAverageCount = 0;
    loadKgAverageIndex = 0;
    loadKgAverageSum = 0.0f;
    for (uint8_t i = 0; i < kLoadRawAverageWindowMax; ++i) {
        loadKgAverageBuffer[i] = 0.0f;
    }
    if (isfinite(seed)) {
        loadKgAverageBuffer[0] = seed;
        loadKgAverageSum = seed;
        loadKgAverageCount = 1;
        loadKgAverageIndex = (activeWindow > 1) ? 1 : 0;
    }
}

void sortSmallFloatArray(float* values, uint8_t count)
{
    for (uint8_t i = 1; i < count; ++i) {
        const float key = values[i];
        int8_t j = static_cast<int8_t>(i) - 1;
        while (j >= 0 && values[j] > key) {
            values[j + 1] = values[j];
            --j;
        }
        values[j + 1] = key;
    }
}

float medianOfArray(float* values, uint8_t count)
{
    if (count == 0) {
        return 0.0f;
    }
    sortSmallFloatArray(values, count);
    const uint8_t mid = count / 2;
    if ((count & 1U) == 0U) {
        return 0.5f * (values[mid - 1] + values[mid]);
    }
    return values[mid];
}

float applyLoadRawMovingAverage(float sample)
{
    if (!isfinite(sample)) {
        return sample;
    }

    const uint8_t configuredWindow = getConfiguredLoadRawAverageWindow();
    if (configuredWindow != loadRawAverageActiveWindow) {
        resetLoadRawAverage(sample);
    }

    const uint8_t activeWindow = loadRawAverageActiveWindow;

    if (loadRawAverageCount < activeWindow) {
        loadRawAverageBuffer[loadRawAverageIndex] = sample;
        loadRawAverageSum += sample;
        ++loadRawAverageCount;
    } else {
        loadRawAverageSum -= loadRawAverageBuffer[loadRawAverageIndex];
        loadRawAverageBuffer[loadRawAverageIndex] = sample;
        loadRawAverageSum += sample;
    }

    ++loadRawAverageIndex;
    if (loadRawAverageIndex >= activeWindow) {
        loadRawAverageIndex = 0;
    }

    return loadRawAverageSum / static_cast<float>(loadRawAverageCount);
}

float applyLoadKgMovingAverage(float sample)
{
    if (!isfinite(sample)) {
        return sample;
    }

    const uint8_t configuredWindow = getConfiguredLoadKgAverageWindow();
    if (configuredWindow != loadKgAverageActiveWindow) {
        resetLoadKgAverage(sample);
    }

    const uint8_t activeWindow = loadKgAverageActiveWindow;

    if (loadKgAverageCount < activeWindow) {
        loadKgAverageBuffer[loadKgAverageIndex] = sample;
        loadKgAverageSum += sample;
        ++loadKgAverageCount;
    } else {
        loadKgAverageSum -= loadKgAverageBuffer[loadKgAverageIndex];
        loadKgAverageBuffer[loadKgAverageIndex] = sample;
        loadKgAverageSum += sample;
    }

    ++loadKgAverageIndex;
    if (loadKgAverageIndex >= activeWindow) {
        loadKgAverageIndex = 0;
    }

    return loadKgAverageSum / static_cast<float>(loadKgAverageCount);
}

float applyLoadRawAverage(float sample)
{
    return applyLoadRawMovingAverage(sample);
}

void resetTachoRawAverage(float seed)
{
    tachoRawAverageCount = 0;
    tachoRawAverageIndex = 0;
    tachoRawAverageSum = 0.0f;

    for (uint8_t i = 0; i < kTachoRawAverageWindow; ++i) {
        tachoRawAverageBuffer[i] = 0.0f;
    }

    if (isfinite(seed)) {
        tachoRawAverageBuffer[0] = seed;
        tachoRawAverageSum = seed;
        tachoRawAverageCount = 1;
        tachoRawAverageIndex = 1;
    }
}

float applyTachoRawAverage(float sample)
{
    if (!isfinite(sample)) {
        return sample;
    }

    if (tachoRawAverageCount < kTachoRawAverageWindow) {
        tachoRawAverageBuffer[tachoRawAverageIndex] = sample;
        tachoRawAverageSum += sample;
        ++tachoRawAverageCount;
    } else {
        tachoRawAverageSum -= tachoRawAverageBuffer[tachoRawAverageIndex];
        tachoRawAverageBuffer[tachoRawAverageIndex] = sample;
        tachoRawAverageSum += sample;
    }

    ++tachoRawAverageIndex;
    if (tachoRawAverageIndex >= kTachoRawAverageWindow) {
        tachoRawAverageIndex = 0;
    }

    return tachoRawAverageSum / static_cast<float>(tachoRawAverageCount);
}

void resetMassflowRawAverage(float seed)
{
    massflowRawAverageCount = 0;
    massflowRawAverageIndex = 0;
    massflowRawAverageSum = 0.0f;

    for (uint8_t i = 0; i < kAuxRawAverageWindow; ++i) {
        massflowRawAverageBuffer[i] = 0.0f;
    }

    if (isfinite(seed)) {
        massflowRawAverageBuffer[0] = seed;
        massflowRawAverageSum = seed;
        massflowRawAverageCount = 1;
        massflowRawAverageIndex = 1;
    }
}

float applyMassflowRawAverage(float sample)
{
    if (!isfinite(sample)) {
        return sample;
    }

    if (massflowRawAverageCount < kAuxRawAverageWindow) {
        massflowRawAverageBuffer[massflowRawAverageIndex] = sample;
        massflowRawAverageSum += sample;
        ++massflowRawAverageCount;
    } else {
        massflowRawAverageSum -= massflowRawAverageBuffer[massflowRawAverageIndex];
        massflowRawAverageBuffer[massflowRawAverageIndex] = sample;
        massflowRawAverageSum += sample;
    }

    ++massflowRawAverageIndex;
    if (massflowRawAverageIndex >= kAuxRawAverageWindow) {
        massflowRawAverageIndex = 0;
    }

    return massflowRawAverageSum / static_cast<float>(massflowRawAverageCount);
}

void resetLambdaRawAverage(float seed)
{
    lambdaRawAverageCount = 0;
    lambdaRawAverageIndex = 0;
    lambdaRawAverageSum = 0.0f;

    for (uint8_t i = 0; i < kAuxRawAverageWindow; ++i) {
        lambdaRawAverageBuffer[i] = 0.0f;
    }

    if (isfinite(seed)) {
        lambdaRawAverageBuffer[0] = seed;
        lambdaRawAverageSum = seed;
        lambdaRawAverageCount = 1;
        lambdaRawAverageIndex = 1;
    }
}

float applyLambdaRawAverage(float sample)
{
    if (!isfinite(sample)) {
        return sample;
    }

    if (lambdaRawAverageCount < kAuxRawAverageWindow) {
        lambdaRawAverageBuffer[lambdaRawAverageIndex] = sample;
        lambdaRawAverageSum += sample;
        ++lambdaRawAverageCount;
    } else {
        lambdaRawAverageSum -= lambdaRawAverageBuffer[lambdaRawAverageIndex];
        lambdaRawAverageBuffer[lambdaRawAverageIndex] = sample;
        lambdaRawAverageSum += sample;
    }

    ++lambdaRawAverageIndex;
    if (lambdaRawAverageIndex >= kAuxRawAverageWindow) {
        lambdaRawAverageIndex = 0;
    }

    return lambdaRawAverageSum / static_cast<float>(lambdaRawAverageCount);
}

// ESP32-S3 ADC map provided for this hardware revision.
constexpr uint8_t kRpmSetpointPin = 1; // ADC1_CH0
constexpr uint8_t kThrottlePotPin = 2; // ADC1_CH1
constexpr uint8_t kTachoPin = 3;       // ADC1_CH2
constexpr uint8_t kKpPotPin = 6;       // swapped with massflow per latest wiring
constexpr uint8_t kMassflowPin = 8;    // kept off GPIO5 to avoid CAN RX conflict
constexpr uint8_t kLambdaPin = 7;      // ADC1_CH6
constexpr uint8_t kLoadCellPin = 32;   // Load-cell analog input
// Drum RPM is derived from tachogen × (1/virtGearRatio)

static float kpPotFilteredAdc = -1.0f;
static float lastAppliedKp = -1.0f;

// Digital inputs
static bool prevSwState = false;
static uint32_t swDebounceMs = 0;
constexpr uint32_t kSwDebounceThresholdMs = 30;

float readAdcSafe(uint8_t pin)
{
    if (digitalPinToAnalogChannel(pin) < 0) {
        static uint32_t lastWarnMs = 0;
        const uint32_t now = millis();
        if (now - lastWarnMs > 3000) {
            lastWarnMs = now;
            Serial.printf("[Input] Invalid ADC pin on this board: %u\n", pin);
        }
        return 0.0f;
    }
    return static_cast<float>(analogRead(pin));
}

float readTachoRpm()
{
    float v = readAdcSafe(kTachoPin);
    v = applyTachoRawAverage(v);
    return v * tachoCal;
}

float readDrumRpm()
{
    // Drum RPM is derived from the tachogen by inverting the configured gear ratio.
    // virtGearRatio = engineRpm / drumRpm, so drumRpm = tachoRpm / virtGearRatio.
    const float ratio = MetaSense::Settings::virtGearRatio;
    return (ratio > 0.01f) ? (tachoRpm / ratio) : tachoRpm;
}
float readLoadKg()
{
    if (loadCellNauReady) {
        return getLoadCellSamplerAverageRaw();
    }

    // Backward-compatible fallback for boards wired without NAU7802.
    return readAdcSafe(kLoadCellPin) * kLoadCellRawScale;
}

uint16_t getLoadCellSampleRateSpsLocal()
{
    return loadCellCurrentRateSps;
}

bool readDirectLoadRawSample(float& outRaw, uint16_t waitMs = 0)
{
    const uint32_t start = millis();

    while (true) {
        if (loadCellNauReady) {
            if (loadCellMutex != nullptr) {
                if (xSemaphoreTake(loadCellMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                    const bool i2cLocked = MetaSense::I2cBus::take(pdMS_TO_TICKS(2));
                    const bool hasSample = loadCellNau.available();
                    if (hasSample) {
                        outRaw = static_cast<float>(loadCellNau.read()) * kLoadCellRawScale;
                    }
                    if (i2cLocked) {
                        MetaSense::I2cBus::give();
                    }
                    xSemaphoreGive(loadCellMutex);
                    if (hasSample) {
                        return isfinite(outRaw);
                    }
                }
            } else if (loadCellNau.available()) {
                outRaw = static_cast<float>(loadCellNau.read()) * kLoadCellRawScale;
                return isfinite(outRaw);
            }
        } else {
            // Keep fallback calibration units aligned with runtime fallback path.
            outRaw = readAdcSafe(kLoadCellPin) * kLoadCellRawScale;
            return isfinite(outRaw);
        }

        if (waitMs == 0 || (millis() - start) >= waitMs) {
            return false;
        }

        delay(2);
    }
}

bool readAveragedDirectLoadRawSample(float& outRaw,
                                     uint8_t burstSamples = kLoadRawBurstSamples,
                                     uint16_t perSampleWaitMs = 20)
{
    float sum = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 0; i < burstSamples; ++i) {
        float raw = 0.0f;
        if (!readDirectLoadRawSample(raw, perSampleWaitMs)) {
            continue;
        }
        if (!isfinite(raw)) {
            continue;
        }
        sum += raw;
        count++;
    }

    if (count == 0) {
        return false;
    }

    outRaw = sum / static_cast<float>(count);
    return true;
}

bool applyLoadCellNauProfile(NAU7802_Gain gain, NAU7802_SampleRate rate, const char* reason)
{
    if (!loadCellNauReady) {
        return false;
    }

    const uint16_t targetSps = nauSampleRateToSps(rate);
    const uint8_t targetGain = nauGainToValue(gain);
    if (targetSps == 0) {
        return false;
    }
    if (targetGain == 0) {
        return false;
    }

    const NauTransitionProfile* profile = findNauTransitionProfile(gain, rate);

    if (loadCellMutex != nullptr) {
        if (xSemaphoreTake(loadCellMutex, pdMS_TO_TICKS(30)) != pdTRUE) {
            return false;
        }
    }

    const bool i2cLocked = MetaSense::I2cBus::take(pdMS_TO_TICKS(30));

    loadCellNau.setGain(gain);
    bool rateBitsOk = false;
    bool gainBitsOk = false;
    uint8_t discarded = 0;

    bool ctrl1Ok = false;
    const uint8_t ctrl1 = loadCellNau.readRegister(NAU7802_CTRL1, &ctrl1Ok);
    if (ctrl1Ok) {
        gainBitsOk = ((ctrl1 & 0x07) == (static_cast<uint8_t>(gain) & 0x07));
    }

    loadCellNau.setRate(rate);

    bool ctrl2Ok = false;
    const uint8_t ctrl2 = loadCellNau.readRegister(NAU7802_CTRL2, &ctrl2Ok);
    if (ctrl2Ok) {
        const uint8_t appliedRateBits = static_cast<uint8_t>((ctrl2 >> 4) & 0x07);
        rateBitsOk = (appliedRateBits == (static_cast<uint8_t>(rate) & 0x07));
    }

    // Restart the conversion cycle after CRS changes so the digital path
    // is synchronized to the new decimation settings.
    bool puOk = false;
    const uint8_t puCtrl = loadCellNau.readRegister(NAU7802_PU_CTRL, &puOk);
    if (puOk) {
        const uint8_t puNoCs = static_cast<uint8_t>(puCtrl & ~NAU7802_PU_CTRL_CS);
        (void)loadCellNau.writeRegister(NAU7802_PU_CTRL, puNoCs);
        delay(kNauRateSwitchRestartDelayMs);
        (void)loadCellNau.writeRegister(NAU7802_PU_CTRL, static_cast<uint8_t>(puNoCs | NAU7802_PU_CTRL_CS));
    }

    const uint32_t settleMs = (profile != nullptr)
        ? profile->settleMs
        : min<uint32_t>(
            kNauRateSwitchMaxSettleMs,
            max<uint32_t>(kNauRateSwitchMinSettleMs, (3000UL + targetSps - 1) / targetSps));
    const uint8_t targetDiscardSamples = (profile != nullptr)
        ? profile->discardSamples
        : min<uint8_t>(
            kNauRateSwitchMaxDiscardSamples,
            max<uint8_t>(kNauRateSwitchMinDiscardSamples,
                static_cast<uint8_t>((settleMs * targetSps + 999UL) / 1000UL)));
    const uint32_t settleDeadline = millis() + settleMs;
    while (discarded < targetDiscardSamples && static_cast<int32_t>(settleDeadline - millis()) > 0) {
        if (loadCellNau.available()) {
            (void)loadCellNau.read();
            ++discarded;
        } else {
            delay(1);
        }
    }

    if (loadCellMutex != nullptr) {
        xSemaphoreGive(loadCellMutex);
    }
    if (i2cLocked) {
        MetaSense::I2cBus::give();
    }

    loadCellCurrentRateSps = targetSps;
    loadCellCurrentGainValue = targetGain;

    Serial.printf("[Input] NAU gain x%u rate %u SPS (%s), ctrl1_ok=%d ctrl2_ok=%d discard=%u/%u settle=%lums entry=%d\n",
                  static_cast<unsigned>(targetGain),
                  static_cast<unsigned>(targetSps),
                  reason != nullptr ? reason : "n/a",
                  gainBitsOk ? 1 : 0,
                  rateBitsOk ? 1 : 0,
                  static_cast<unsigned>(discarded),
                  static_cast<unsigned>(targetDiscardSamples),
                  static_cast<unsigned long>(settleMs),
                  profile != nullptr ? 1 : 0);

    return gainBitsOk && rateBitsOk;
}

void sortFloatArray(float* values, uint8_t count)
{
    for (uint8_t i = 1; i < count; ++i) {
        const float key = values[i];
        int8_t j = static_cast<int8_t>(i) - 1;
        while (j >= 0 && values[j] > key) {
            values[j + 1] = values[j];
            --j;
        }
        values[j + 1] = key;
    }
}

bool computeTrimmedStats(const float* samples,
                         uint8_t count,
                         float& outAverage,
                         float& outMaxAbsDeviation)
{
    if (count == 0) {
        return false;
    }

    float sorted[96];
    if (count > sizeof(sorted) / sizeof(sorted[0])) {
        return false;
    }

    for (uint8_t i = 0; i < count; ++i) {
        sorted[i] = samples[i];
    }

    sortFloatArray(sorted, count);

    const uint8_t trimEachSide = (count >= 10) ? max<uint8_t>(1, count / 10) : 0;
    const uint8_t startIndex = trimEachSide;
    const uint8_t endIndex = count - trimEachSide;
    if (startIndex >= endIndex) {
        return false;
    }

    float sum = 0.0f;
    uint8_t usedCount = 0;
    for (uint8_t i = startIndex; i < endIndex; ++i) {
        sum += sorted[i];
        usedCount++;
    }
    if (usedCount == 0) {
        return false;
    }

    outAverage = sum / static_cast<float>(usedCount);

    float maxDev = 0.0f;
    for (uint8_t i = startIndex; i < endIndex; ++i) {
        const float dev = fabsf(sorted[i] - outAverage);
        if (dev > maxDev) {
            maxDev = dev;
        }
    }
    outMaxAbsDeviation = maxDev;
    return true;
}

bool getLoadCellSamplerStats(float& outAverageRaw, float& outMaxAbsDeviation)
{
    float samples[kLoadCellSamplerWindow];
    uint8_t count = 0;

    portENTER_CRITICAL(&loadCellSamplerMux);
    count = loadCellSamplerCount;
    for (uint8_t i = 0; i < count; ++i) {
        samples[i] = loadCellSamplerBuffer[i];
    }
    portEXIT_CRITICAL(&loadCellSamplerMux);

    if (count == 0) {
        return false;
    }

    return computeTrimmedStats(samples, count, outAverageRaw, outMaxAbsDeviation);
}

bool sampleLoadRawStats(float& outAverageRaw, float& outMaxAbsDeviation, uint16_t timeoutMs = 220, uint8_t maxSamples = 24)
{
    float samples[kLoadRawStatsMaxSamples];
    uint8_t count = 0;
    const uint8_t targetSamples = min<uint8_t>(maxSamples, kLoadRawStatsMaxSamples);
    const uint32_t start = millis();

    float seedRaw = filteredAdc;
    if (!isfinite(seedRaw)) {
        seedRaw = getLoadCellSamplerAverageRaw();
    }
    resetLoadRawAverage(seedRaw);

    while ((millis() - start) < timeoutMs && count < targetSamples) {
        float raw = 0.0f;
        if (readAveragedDirectLoadRawSample(raw, kLoadRawBurstSamples, 12) && isfinite(raw)) {
            samples[count++] = applyLoadRawAverage(raw);
        }
        delay(5);
    }

    if (count == 0) {
        return false;
    }

    return computeTrimmedStats(samples, count, outAverageRaw, outMaxAbsDeviation);
}

void loadCellSamplerTask(void* /*pvParameters*/)
{
    for (;;) {
        const uint32_t startedUs = micros();
        if (!loadCellExclusiveSampling) {
            if (loadCellNauReady) {
                float rawSamples[8];
                uint8_t rawCount = 0;

                if (loadCellMutex != nullptr && xSemaphoreTake(loadCellMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                    while (loadCellNau.available() && rawCount < 8) {
                        rawSamples[rawCount++] = static_cast<float>(loadCellNau.read()) * kLoadCellRawScale;
                    }
                    xSemaphoreGive(loadCellMutex);
                }

                for (uint8_t i = 0; i < rawCount; ++i) {
                    pushLoadCellSamplerRaw(rawSamples[i]);
                }
            } else {
                const float raw = readAdcSafe(kLoadCellPin) * kLoadCellRawScale;
                pushLoadCellSamplerRaw(raw);
            }
        }

        const uint32_t elapsedUs = micros() - startedUs;
        loadCellSamplerLastUs = elapsedUs;
        if (elapsedUs > loadCellSamplerMaxUs) {
            loadCellSamplerMaxUs = elapsedUs;
        }
        if (loadCellSamplerEmaUs == 0) {
            loadCellSamplerEmaUs = elapsedUs;
        } else {
            loadCellSamplerEmaUs = (loadCellSamplerEmaUs * 7U + elapsedUs) / 8U;
        }
        ++loadCellSamplerLoops;

        vTaskDelay(pdMS_TO_TICKS(kLoadCellSamplerPeriodMs));
    }
}

void tryInitLoadCellNau()
{
    if (loadCellNauReady) {
        return;
    }

    const bool i2cLocked = MetaSense::I2cBus::take(pdMS_TO_TICKS(100));
    loadCellNauReady = i2cLocked && loadCellNau.begin(&Wire);
    if (i2cLocked) {
        MetaSense::I2cBus::give();
    }
    if (loadCellNauReady) {
        loadCellNauLdoConfigured = false;
        loadCellNauInternalCalOk = false;
        loadCellNauInternalCalAttempts = 0;
        if (!loadCellNau.resetAndPowerUp()) {
            Serial.println("[Input] Warning: NAU explicit reset/power-up after begin failed");
        }

        const bool ldoConfigured = loadCellNau.setLDO(kLoadCellDefaultLdo);
        loadCellNauLdoConfigured = ldoConfigured;
        if (!ldoConfigured) {
            Serial.println("[Input] Warning: NAU internal LDO configuration failed");
        } else {
            delay(kNauLdoRampDelayMs);
        }

        if (!applyLoadCellNauProfile(NAU7802_GAIN_128, NAU7802_RATE_80SPS, "init_fixed")) {
            Serial.println("[Input] Warning: NAU fixed gain/rate verification failed during init");
        }

        // Flush first conversions after startup/config change before AFE calibration.
        for (uint8_t i = 0; i < kNauInitFlushReadings; ++i) {
            uint32_t waitStart = millis();
            while (!loadCellNau.available() && (millis() - waitStart) < 50U) {
                delay(1);
            }
            if (loadCellNau.available()) {
                (void)loadCellNau.read();
            }
        }

        bool internalCalOk = false;
        for (uint8_t attempt = 1; attempt <= kNauInternalCalMaxAttempts; ++attempt) {
            loadCellNauInternalCalAttempts = attempt;
            internalCalOk = loadCellNau.calibrate(NAU7802_CALMOD_INTERNAL, 1200);
            if (internalCalOk) {
                break;
            }
            Serial.printf("[Input] NAU internal offset calibration failed (attempt %u/%u)\n",
                          static_cast<unsigned>(attempt),
                          static_cast<unsigned>(kNauInternalCalMaxAttempts));
            if (attempt < kNauInternalCalMaxAttempts) {
                delay(kNauInternalCalRetryDelayMs);
            }
        }
        loadCellNauInternalCalOk = internalCalOk;
        if (!internalCalOk) {
            Serial.println("[Input] Warning: NAU internal AFE calibration failed");
        }

        Serial.printf("[Input] Load-cell ADC source ready (NAU7802 @ 0x2A, ldo=%d, cal=%d)\n",
                      ldoConfigured ? 1 : 0,
                      internalCalOk ? 1 : 0);
        Serial0.printf("[Input] Load-cell ADC source ready (NAU7802 @ 0x2A, ldo=%d, cal=%d)\n",
                       ldoConfigured ? 1 : 0,
                       internalCalOk ? 1 : 0);
    } else {
        Serial.println("[Input] Load-cell ADC source unavailable (NAU7802), using GPIO32 ADC fallback");
        Serial0.println("[Input] Load-cell ADC source unavailable (NAU7802), using GPIO32 ADC fallback");
    }
}

void setLoadCellStableSampling(bool enable)
{
    (void)enable;
    if (!loadCellNauReady) {
        return;
    }

    float seedRaw = getLoadCellSamplerAverageRaw();
    if (!isfinite(seedRaw)) {
        seedRaw = filteredAdc;
    }
    resetLoadCellSampler(seedRaw);
}

void updateAmbientInputs(bool forceSample)
{
    if (!ambientBmeReady) {
        return;
    }

    const uint32_t now = millis();

    // Complete any in-flight forced conversion without blocking the loop.
    if (ambientBmeReadPending) {
        const bool readyToFinish = forceSample ||
            (static_cast<int32_t>(now - ambientBmeReadReadyMs) >= 0);
        if (readyToFinish) {
            if (ambientBme.endReading()) {
                const float bmeTemp = ambientBme.temperature;
                const float bmeHumidity = ambientBme.humidity;
                const float pressurePa = ambientBme.pressure;

                if (isfinite(bmeTemp) && bmeTemp > -50.0f && bmeTemp < 120.0f) {
                    ambientTempC = bmeTemp;
                }
                if (isfinite(bmeHumidity)) {
                    float humidityWithOffset = bmeHumidity + MetaSense::Settings::ambientRhOffsetPct;
                    if (humidityWithOffset < 0.0f) humidityWithOffset = 0.0f;
                    if (humidityWithOffset > 100.0f) humidityWithOffset = 100.0f;
                    ambientHumidityPct = humidityWithOffset;
                }
                if (isfinite(pressurePa) && pressurePa > 10000.0f && pressurePa < 120000.0f) {
                    ambientPressureHpa = pressurePa / 100.0f;
                }
            }
            ambientBmeReadPending = false;
            lastAmbientSampleMs = now;
        }
    }

    if (ambientBmeReadPending) {
        return;
    }

    if (!forceSample && (now - lastAmbientSampleMs) < kAmbientSamplePeriodMs) {
        return;
    }

    const uint32_t readyAtMs = ambientBme.beginReading();
    if (readyAtMs != 0) {
        ambientBmeReadPending = true;
        ambientBmeReadReadyMs = readyAtMs;
    }
}

float readEgtHotC()
{
    if (!egtDigital.isReady()) {
        return 0.0f;
    }

    const float egt = egtDigital.readHotC();
    if (!isfinite(egt) || egt < 0.0f || egt > 1800.0f) {
        return 0.0f;
    }

    return egt;
}

float readEgtAmbientC()
{
    if (!egtDigital.isReady()) {
        return 0.0f;
    }

    const float ambient = egtDigital.readAmbientC();
    if (!isfinite(ambient) || ambient < -50.0f || ambient > 200.0f) {
        return 0.0f;
    }

    return ambient;
}

float readAmbientC()    { updateAmbientInputs(false); return ambientTempC; }
float readPressureHpa() { updateAmbientInputs(false); return ambientPressureHpa; }
float computeAirDensityKgM3(float tempC, float pressureHpa, float humidityPct)
{
    // Magnus formula for saturation vapour pressure (hPa)
    const float Psat = 6.1078f * expf(17.269f * tempC / (237.3f + tempC));
    const float Pv   = (humidityPct / 100.0f) * Psat;  // partial vapour pressure, hPa
    const float Pd   = pressureHpa - Pv;               // dry-air partial pressure, hPa
    const float T_K  = tempC + 273.15f;
    // Ideal gas: ρ = Pd/(Rd*T) + Pv/(Rv*T)
    const float Pd_Pa = Pd * 100.0f;
    const float Pv_Pa = Pv * 100.0f;
    const float rho = (Pd_Pa / (287.058f * T_K)) + (Pv_Pa / (461.495f * T_K));
    return (rho > 0.0f) ? rho : 1.225f;
}

float readAirDensity()  { return computeAirDensityKgM3(ambientTempC, ambientPressureHpa, ambientHumidityPct); }
float readHumidity()    { updateAmbientInputs(false); return ambientHumidityPct; }
float readETorque()     { return 0.0f; }

void updateI2cScanSummary()
{
    String summary;
    if (MetaSense::I2cBus::take(pdMS_TO_TICKS(50))) {
        for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                if (!summary.isEmpty()) {
                    summary += ",";
                }
                char buf[6];
                snprintf(buf, sizeof(buf), "0x%02X", addr);
                summary += buf;
            }
        }
        MetaSense::I2cBus::give();
    }
    i2cScanSummary = summary.isEmpty() ? String("none") : summary;
}

void applyRuntimeKpFromPot(bool forceApply)
{
    const float adc = readAdcSafe(kKpPotPin);
    if (kpPotFilteredAdc < 0.0f) {
        kpPotFilteredAdc = adc;
    } else {
        kpPotFilteredAdc = lpFilter(kpPotFilteredAdc, adc, kRuntimeKpAlpha);
    }

    float normalized = kpPotFilteredAdc / 4095.0f;
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    const float kpFromPot = kRuntimeKpMin + normalized * (kRuntimeKpMax - kRuntimeKpMin);
    if (!forceApply && fabsf(kpFromPot - lastAppliedKp) < kRuntimeKpApplyDelta) {
        return;
    }

    MetaSense::ControlTask::configurePI(kpFromPot,
                                        MetaSense::Settings::ki,
                                        TORQUE_MIN,
                                        TORQUE_MAX);
    lastAppliedKp = kpFromPot;
}

float readRpmSetpointPot()
{
    const float adc = readAdcSafe(kRpmSetpointPin);
    float rpmSetpoint = (adc / 4095.0f) * RPM_SETPOINT_MAX;
    if (rpmSetpoint < 0.0f) rpmSetpoint = 0.0f;
    if (rpmSetpoint > RPM_SETPOINT_MAX) rpmSetpoint = RPM_SETPOINT_MAX;
    if (rpmSetpoint < 100.0f) rpmSetpoint = 0.0f;
    return rpmSetpoint;
}

float readThrottlePotPercent()
{
    const float adc = readAdcSafe(kThrottlePotPin);
    float percent = (adc / 4095.0f) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

float readMassflowM3h()
{
    const float adc = readAdcSafe(kMassflowPin);
    float value = (adc / 4095.0f) * kMassflowMaxM3h;
    if (value < 0.0f) value = 0.0f;
    if (value > kMassflowMaxM3h) value = kMassflowMaxM3h;
    return applyMassflowRawAverage(value);
}

float readLambdaValue()
{
    const float adc = readAdcSafe(kLambdaPin);
    float value = kLambdaMin + (adc / 4095.0f) * (kLambdaMax - kLambdaMin);
    if (value < kLambdaMin) value = kLambdaMin;
    if (value > kLambdaMax) value = kLambdaMax;
    return applyLambdaRawAverage(value);
}

// Persistent angular velocity for inertia differentiation.
static float omegaPrev = 0.0f;

// Delta transmission: only send fields that changed to reduce bandwidth
static MetaSense::Telemetry prevTelemetrySnapshot;
static bool prevTelemetryInitialized = false;
static bool prevRpmDeltaError = false;
static bool prevCanFallbackActive = false;
static bool prevRecordingSent = false;
static bool prevRecordingSentInitialized = false;
static bool prevRpmSourceCanSent = false;
static bool prevRpmSourceCanInitialized = false;
static bool prevEnergyActiveSent = false;
static bool prevEnergyActiveInitialized = false;
static uint8_t prevDynoModeSent = 0;
static bool prevDynoModeInitialized = false;
static bool prevVcuReadySent = false;
static bool prevVcuReadyInitialized = false;
static bool prevSwActiveSent = false;
static bool prevSwActiveInitialized = false;

// Helper: compare floats with tolerance
static inline bool floatChanged(float a, float b, float tolerance = 0.01f)
{
    return fabs(a - b) > tolerance;
}

static float selectLeafActualTorqueNm(const LeafInvFeedback& fb, float demandNm)
{
    const float candidates[4] = {
        static_cast<float>(fb.torque_raw01_le) * 0.5f,
        static_cast<float>(fb.torque_raw01_be) * 0.5f,
        static_cast<float>(fb.torque_raw23_le) * 0.5f,
        static_cast<float>(fb.torque_raw23_be) * 0.5f
    };

    float best = 0.0f;
    float bestScore = 1.0e9f;
    const bool useDemandReference = fabsf(demandNm) >= 2.0f;

    for (int i = 0; i < 4; ++i) {
        const float v = candidates[i];
        if (!isfinite(v) || fabsf(v) > 500.0f) {
            continue;
        }
        const float score = useDemandReference ? fabsf(v - demandNm) : fabsf(v);
        if (score < bestScore) {
            bestScore = score;
            best = v;
        }
    }

    return best;
}

void pollLeafCanFrames(uint32_t nowMs)
{
#if METASENSE_LEAF_CAN_RX_ENABLED
    static bool canConfigured = false;
    static bool canReadyLogged = false;

    if (!canConfigured) {
        MetaSense::CANBus::configure(kLeafCanConfig);
        canConfigured = true;
    }

    MetaSense::CANBus::poll(nowMs);

    if (!MetaSense::CANBus::isReady()) {
        canReadyLogged = false;
        return;
    }

    if (!canReadyLogged) {
        Serial.println("[Input] Leaf CAN RX initialized");
        Serial0.println("[Input] Leaf CAN RX initialized");
        canReadyLogged = true;
    }

    const LeafInvFeedback& leafFb = MetaSense::CANBus::feedback();
    if (!s_leafCanPartnerSeen &&
        (leafFb.rpm_frames > 0U ||
         leafFb.torque_frames > 0U ||
         leafFb.temps_frames > 0U ||
         leafFb.status_frames > 0U)) {
        s_leafCanPartnerSeen = true;
    }
    if (leafFb.rpm_update_ms != 0U && leafFb.rpm_update_ms != lastCanRpmFrameMs) {
        s_leafSimFeedbackActive = false;
        // Keep Leaf CAN RPM available for monitor display, independent of control source.
        leafCanRpmMonitor = leafFb.rpm;
        lastCanRpmMonitorUpdate = leafFb.rpm_update_ms;
#if !METASENSE_FORCE_TACHO_RPM_SOURCE
        MetaSense::Input::updateCanRpm(leafFb.rpm);
#endif
        lastCanLeafAnyUpdate = leafFb.rpm_update_ms;
        lastCanRpmFrameMs = leafFb.rpm_update_ms;

        if (kLeafCanHandshakeOnFirst1da && !s_leafHandshakeSent && !s_leafHandshakeArmed) {
            s_leafHandshakeArmed = true;
            s_leafHandshakeSentCount = 0;
            s_leafHandshakeAttemptCount = 0;
            s_leafHandshakeStartMs = nowMs;
            s_leafHandshakeLastAttemptMs = 0;
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
            Serial.println("[VCM-HS] First 0x1DA observed, starting extended 0x55B handshake campaign");
            Serial0.println("[VCM-HS] First 0x1DA observed, starting extended 0x55B handshake campaign");
#endif
        }
    }
    if (leafFb.torque_update_ms != 0U && leafFb.torque_update_ms != lastCanTorqueFrameMs) {
        s_leafSimFeedbackActive = false;
        MetaSense::Input::updateCanTorque(leafFb.torque_nm);
        lastCanLeafAnyUpdate = leafFb.torque_update_ms;
        lastCanTorqueFrameMs = leafFb.torque_update_ms;
    }
    if (leafFb.temps_update_ms != 0U && leafFb.temps_update_ms != lastCanTempsFrameMs) {
        s_leafSimFeedbackActive = false;
        MetaSense::Input::updateCanTemps(leafFb.inverter_temp, leafFb.stator_temp, leafFb.coolant_temp);
        lastCanLeafAnyUpdate = leafFb.temps_update_ms;
        lastCanTempsFrameMs = leafFb.temps_update_ms;
    }
    if (leafFb.status_update_ms != 0U && leafFb.status_update_ms != lastCanStatusFrameMs) {
        s_leafSimFeedbackActive = false;
        MetaSense::Input::updateCanStatus(leafFb.ready, leafFb.fault, leafFb.warning, leafFb.limp);
        lastCanLeafAnyUpdate = leafFb.status_update_ms;
        lastCanStatusFrameMs = leafFb.status_update_ms;
    }
#else
    (void)nowMs;
#endif
}

void maybeInjectLeafSimFeedback(uint32_t nowMs)
{
    if (!METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS || !MetaSense::Settings::leafSimFeedbackEnabled) {
        s_leafSimFeedbackActive = false;
        s_leafSimLogPrinted = false;
        return;
    }

    const uint32_t lastRealFrameMs = max(max(lastCanRpmFrameMs, lastCanTorqueFrameMs),
                                         max(lastCanTempsFrameMs, lastCanStatusFrameMs));
    const bool realPartnerFresh = (lastRealFrameMs != 0U) &&
                                  ((nowMs - lastRealFrameMs) <= kLeafVcmFeedbackTimeoutMs);
    if (realPartnerFresh) {
        return;
    }

    if (nowMs < kLeafSimBootDelayMs) {
        return;
    }

    if ((nowMs - s_leafSimLastInjectMs) < kLeafSimFeedbackPeriodMs) {
        return;
    }

    s_leafSimLastInjectMs = nowMs;
    s_leafSimFeedbackActive = true;

    // Simulate Leaf feedback path on bench setups without inverter on the CAN bus.
    float tachEngineRpm = tachoRpm;
    if (!isfinite(tachEngineRpm) || tachEngineRpm < 0.0f) {
        tachEngineRpm = readTachoRpm();
    }
    const float ratio = (MetaSense::Settings::virtGearRatio > 0.01f)
        ? MetaSense::Settings::virtGearRatio
        : 1.0f;
    const float emotorRpm = (tachEngineRpm > 0.0f) ? (tachEngineRpm / ratio) : 0.0f;

    MetaSense::Input::updateCanRpm(emotorRpm);
    MetaSense::Input::updateCanTorque(0.0f);
    MetaSense::Input::updateCanTemps(25.0f, 25.0f, 25.0f);
    const bool simInverterReady = (nowMs >= kLeafSimReadyDelayMs);
    MetaSense::Input::updateCanStatus(simInverterReady, false, false, false);

    if (!s_leafSimLogPrinted) {
        Serial.println("[VCM-SIM] Injecting Leaf CAN feedback (no inverter on bus)");
        Serial0.println("[VCM-SIM] Injecting Leaf CAN feedback (no inverter on bus)");
        s_leafSimLogPrinted = true;
    }
}

void resetFallbackRunLog(uint32_t nowMs)
{
    fallbackRunCount = 0;
    fallbackRunStartMs = nowMs;
    fallbackRunLastSampleMs = 0;
}

void appendFallbackRunLogSample(uint32_t nowMs)
{
    if (fallbackRunCount >= kFallbackRunMaxPoints) {
        return;
    }
    if (fallbackRunLastSampleMs != 0 && (nowMs - fallbackRunLastSampleMs) < kFallbackRunSamplePeriodMs) {
        return;
    }

    const size_t idx = fallbackRunCount++;
    fallbackRunRpm[idx] = tele.rpm;
    fallbackRunKw[idx] = tele.kw;
    fallbackRunTorque[idx] = tele.torqueNm;
    fallbackRunTimeSec[idx] = (nowMs >= fallbackRunStartMs)
        ? static_cast<float>(nowMs - fallbackRunStartMs) / 1000.0f
        : 0.0f;
    fallbackRunLastSampleMs = nowMs;
}

void commitFallbackRunLog(const char* reason)
{
    if (fallbackRunCount == 0) {
        const uint32_t nowMs = millis();
        resetFallbackRunLog(nowMs);
        appendFallbackRunLogSample(nowMs);
    }

    String payload;
    payload.reserve(4096);
    payload = "{\"customer\":\"\",\"unit\":\"\",\"comments\":\"\",\"meta\":{";
    payload += "\"reason\":\"" + String((reason != nullptr) ? reason : "firmware_fallback") + "\",";
    payload += "\"xAxis\":\"time\",";
    payload += "\"mode\":\"firmware\",";
    payload += "\"session\":0,";
    payload += "\"samples\":" + String(static_cast<unsigned long>(fallbackRunCount));
    payload += "},\"peaks\":{";
    payload += "\"hp\":" + String(tele.peakKW, 2) + ",";
    payload += "\"hp_rpm\":" + String(tele.peakKW_RPM, 0) + ",";
    payload += "\"torque\":" + String(tele.peakTorque, 2) + ",";
    payload += "\"torque_rpm\":" + String(tele.peakTorque_RPM, 0) + ",";
    payload += "\"egt\":" + String(tele.egtHotC, 1);
    payload += "},\"points\":[";

    for (size_t i = 0; i < fallbackRunCount; ++i) {
        if (i > 0) payload += ",";
        payload += "{\"r\":" + String(fallbackRunRpm[i], 1);
        payload += ",\"x\":" + String(fallbackRunTimeSec[i], 2);
        payload += ",\"h\":" + String(fallbackRunKw[i], 2);
        payload += ",\"t\":" + String(fallbackRunTorque[i], 2);
        payload += "}";
    }
    payload += "]}";

    (void)MetaSense::RunStorage::saveRun(payload);
}

void updateDyno(MetaSense::Telemetry& t, float dtSec)
{
    if (MetaSense::Settings::inertiaMode) {
        // --- Inertia dyno path ---
        // Use drum/e-motor RPM with physical drum inertia in inertia mode.
        const float omegaDrum = t.drumRpm * (2.0f * 3.14159265f / 60.0f);
        const float ratio = (MetaSense::Settings::virtGearRatio > 0.01f)
            ? MetaSense::Settings::virtGearRatio
            : 1.0f;

        // Angular acceleration α = dω/dt, lightly filtered
        float alpha = 0.0f;
        if (dtSec > 0.001f) {
            alpha = lpFilter(0.0f, (omegaDrum - omegaPrev) / dtSec,
                             MetaSense::Settings::filterAlpha);
        }
        omegaPrev = omegaDrum;

        // T = J * alpha with measured drum inertia.
        const float J = MetaSense::Settings::drumInertiaKgM2;
        const float torqueDrum = J * alpha;
        const float torqueEngineEq = torqueDrum / ratio;
        torqueFilt = lpFilter(torqueFilt, torqueEngineEq, MetaSense::Settings::filterAlpha);

        // Inertia mode: report test-engine equivalent torque while preserving physical drum power.
        t.torqueNm      = torqueFilt;
        t.brakeTorqueNm = torqueDrum;

        // Apply climate correction so displayed torque and derived power use corrected values.
        const float rhoActual = computeAirDensityKgM3(ambientTempC, ambientPressureHpa, ambientHumidityPct);
        const float rhoRef    = computeAirDensityKgM3(20.0f, 1013.25f, 0.0f);
        t.climateCF = (rhoActual > 0.3f) ? (rhoRef / rhoActual) : 1.0f;
        t.torqueNm      *= t.climateCF;
        t.brakeTorqueNm *= t.climateCF;

        const float powerW = t.torqueNm * omegaDrum;
        t.kw = powerW / 1000.0f;

        t.energyMJ += (powerW * dtSec) / 1000000.0f;

    } else {
        // --- Brake / load-cell dyno path (original) ---
        const float ratio = (MetaSense::Settings::virtGearRatio > 0.01f)
            ? MetaSense::Settings::virtGearRatio
            : 1.0f;
        const float omegaDrum = t.drumRpm * (2.0f * 3.14159265f / 60.0f);
        const float armConfigured = (MetaSense::Settings::armCm > 0.001f)
            ? MetaSense::Settings::armCm
            : 20.0f;
        // UI stores arm length in cm. Accept legacy meter values (<=2.0) too.
        const float armMeters = (armConfigured <= 2.0f)
            ? armConfigured
            : (armConfigured / 100.0f);
        // Measured brake torque from calibrated load-cell kg and lever arm.
        const float rawBrakeTorqueNm = (t.loadKg * 9.82f) * armMeters;
        // Convert brake/drum torque to test-engine equivalent using RPM ratio.
        const float rawEngineTorqueNm = rawBrakeTorqueNm / ratio;
        t.torqueNm      = rawEngineTorqueNm;
        t.brakeTorqueNm = rawBrakeTorqueNm;

        // --- Climate correction: normalise torque/power to standard conditions ---
        // Reference: 1013.25 hPa, 20 °C, 0 % RH  (ρ_ref ≈ 1.2041 kg/m³)
        const float rhoActual = computeAirDensityKgM3(ambientTempC, ambientPressureHpa, ambientHumidityPct);
        const float rhoRef    = computeAirDensityKgM3(20.0f, 1013.25f, 0.0f);
        t.climateCF = (rhoActual > 0.3f) ? (rhoRef / rhoActual) : 1.0f;
        t.torqueNm      *= t.climateCF;
        t.brakeTorqueNm *= t.climateCF;

        // P(W) = T(Nm) * ω(rad/s) = T * rpm * 2π / 60
        // Correct for drivetrain losses to get crank power.
        const float eff = MetaSense::Settings::drivetrainEff / 100.0f;
        const float omegaEngine = t.rpm * (2.0f * 3.14159265f / 60.0f);
        const float powerW = (eff > 0.01f) ? (t.torqueNm * omegaEngine / eff) : (t.torqueNm * omegaEngine);
        t.kw = powerW / 1000.0f;
        t.energyMJ += (powerW * dtSec) / 1000000.0f;

        omegaPrev = omegaDrum; // keep in sync
    }
}

bool checkSafety(const MetaSense::Telemetry& t)
{
    if (t.rpm > RPM_MAX_LIMIT)       return false;
    if (t.egtHotC > EGT_MAX_LIMIT_C) return false;
    return true;
}

void notifyClients(const MetaSense::Telemetry &data, bool isRecording)
{
    AsyncWebSocket& wsock = MetaSense::WebSocketServer::socket();
    if (wsock.count() == 0) {
        return;
    }

    // Initialize snapshot on first call
    if (!prevTelemetryInitialized) {
        prevTelemetrySnapshot = data;
        prevTelemetryInitialized = true;
    }

    static uint8_t slowTelemetrySlice = 0;
    const uint32_t nowMs = millis();
    const uint8_t slowSliceNow = slowTelemetrySlice;
    slowTelemetrySlice = static_cast<uint8_t>((slowTelemetrySlice + 1U) % kWebSocketSlowTelemetrySlices);
    const bool trendMinimalTelemetry = METASENSE_TREND_MINIMAL_TELEMETRY && gUiModeHintTrend;

#if METASENSE_STREAM_DIAGNOSTICS
    uint16_t wsClients = 0;
    uint16_t wsSentNow = 0;
    uint16_t wsSkipBacklogNow = 0;
    uint16_t wsSkipFullNow = 0;
    uint16_t wsSkipNoSendNow = 0;
#endif

    String json;
    json.reserve(420);

    const float armConfigured = (MetaSense::Settings::armCm > 0.001f)
        ? MetaSense::Settings::armCm
        : 20.0f;
    const float armMeters = (armConfigured <= 2.0f)
        ? armConfigured
        : (armConfigured / 100.0f);
    const float measuredLoadTorqueNm = isfinite(data.loadKg)
        ? (data.loadKg * 9.82f) * armMeters
        : 0.0f;
    const LeafInvFeedback& leafFbDiag = MetaSense::CANBus::feedback();

    json = "{\"type\":\"data\",";

    // Keep a lightweight heartbeat for UI time alignment.
    json += "\"heartbeat_ms\":" + String(nowMs) + ",";

    // Optional stream diagnostics payload (primarily for tuning/debug sessions).
    // Include on every fast frame for real-time monitoring, not just slow cadence.
#if METASENSE_STREAM_DIAGNOSTICS
        json += "\"ws_clients\":" + String(wsock.count()) + ",";
        json += "\"ws_sent_total\":" + String(wsSentTotal) + ",";
        json += "\"ws_skip_backlog_total\":" + String(wsSkipBacklogTotal) + ",";
        json += "\"ws_skip_full_total\":" + String(wsSkipFullTotal) + ",";
        json += "\"ws_skip_nosend_total\":" + String(wsSkipNoSendTotal) + ",";
#if METASENSE_JSON_DELAY_COUNTERS
        json += "\"ws_json_last_len\":" + String(wsJsonLastLen) + ",";
        json += "\"ws_json_max_len\":" + String(wsJsonMaxLen) + ",";
        json += "\"ws_json_over_reserve_total\":" + String(wsJsonOverReserveTotal) + ",";
#endif
#endif

    // Fast telemetry fields (20 Hz)
    if (!trendMinimalTelemetry) {
        if (floatChanged(data.rpm, prevTelemetrySnapshot.rpm, 1.0f)) {
            json += "\"rpm\":" + String(data.rpm, 0) + ",";
        }
        if (rpmDeltaError != prevRpmDeltaError) {
            json += "\"rpm_error\":" + String(rpmDeltaError ? 1 : 0) + ",";
            prevRpmDeltaError = rpmDeltaError;
        }
        if (canFallbackActive != prevCanFallbackActive) {
            json += "\"can_fallback\":" + String(canFallbackActive ? 1 : 0) + ",";
            prevCanFallbackActive = canFallbackActive;
        }
    #if METASENSE_DASHBOARD_TRANSITIONAL_RUN_FIELDS
        if (!prevRpmSourceCanInitialized || activeRpmFromCan != prevRpmSourceCanSent) {
            String rpmSourceStr = String(activeRpmFromCan ? "leafrpm" : "tachogen");
            json += "\"rpm_source_active\":\"" + rpmSourceStr + "\",";
            prevRpmSourceCanSent = activeRpmFromCan;
            prevRpmSourceCanInitialized = true;
        }
    #else
        String rpmSourceStr = String(activeRpmFromCan ? "leafrpm" : "tachogen");
        json += "\"rpm_source_active\":\"" + rpmSourceStr + "\",";
    #endif
        if (floatChanged(data.drumRpm, prevTelemetrySnapshot.drumRpm, 1.0f)) {
            json += "\"drum_rpm\":" + String(data.drumRpm, 0) + ",";
        }
        if (floatChanged(data.rpmTarget, prevTelemetrySnapshot.rpmTarget, 1.0f)) {
            json += "\"rpm_target\":" + String(data.rpmTarget, 0) + ",";
        }
        if (floatChanged(data.kw, prevTelemetrySnapshot.kw, 0.05f)) {
            json += "\"kw\":" + String(data.kw, 2) + ",";
        }
        if (floatChanged(data.peakKW, prevTelemetrySnapshot.peakKW, 0.05f)) {
            json += "\"peakKW\":" + String(data.peakKW, 2) + ",";
        }
        if (floatChanged(data.peakKW_RPM, prevTelemetrySnapshot.peakKW_RPM, 5.0f)) {
            json += "\"peakKW_RPM\":" + String(data.peakKW_RPM, 0) + ",";
        }

        // Dashboard path: keep full torque family + load.
        json += "\"torque\":" + String(data.torqueNm, 2) + ",";
        json += "\"brakeTorque\":" + String(data.brakeTorqueNm, 2) + ",";
        json += "\"torque_measured\":" + String(measuredLoadTorqueNm, 2) + ",";
        json += "\"e_torque\":" + String(data.eTorque, 2) + ",";
        json += "\"load_kg\":" + String(data.loadKg, 1) + ",";
        json += "\"leaf_rpm\":" + String(data.leaf_rpm, 0) + ",";
        json += "\"leaf_torque\":" + String(data.leaf_torqueNm, 2) + ",";
        json += "\"leaf_torque_demand\":" + String(data.leaf_torqueDemandNm, 2) + ",";
        json += "\"leaf_1da_input_v\":" + String(leafFbDiag.input_voltage, 1) + ",";
        json += "\"leaf_1da_torque_nm\":" + String(leafFbDiag.torque_nm, 2) + ",";
        json += "\"leaf_1da_rpm\":" + String(leafFbDiag.rpm, 0) + ",";
        json += "\"leaf_1da_clock\":" + String(static_cast<unsigned long>(leafFbDiag.mg_clock)) + ",";
        json += "\"leaf_1da_err\":" + String(static_cast<unsigned long>(leafFbDiag.mg_error_codes)) + ",";
        json += "\"leaf_1da_crc\":" + String(static_cast<unsigned long>(leafFbDiag.crc_1da)) + ",";
        json += "\"leaf_id1da_frames\":" + String(static_cast<unsigned long>(leafFbDiag.rpm_frames)) + ",";
        json += "\"leaf_id1dc_frames\":" + String(static_cast<unsigned long>(leafFbDiag.temps_frames)) + ",";
        json += "\"leaf_id1da_age_ms\":" + String(static_cast<unsigned long>(elapsedMsSafe(nowMs, leafFbDiag.rpm_update_ms))) + ",";
        json += "\"leaf_id1dc_age_ms\":" + String(static_cast<unsigned long>(elapsedMsSafe(nowMs, leafFbDiag.temps_update_ms))) + ",";
        json += "\"leaf_inv_temp\":" + String(data.leaf_invTempC, 1) + ",";
        json += "\"leaf_stator_temp\":" + String(data.leaf_statorTempC, 1) + ",";
        json += "\"leaf_coolant_temp\":" + String(data.leaf_coolantTempC, 1) + ",";
        json += "\"leaf_ready\":" + String(data.leaf_invReady ? 1 : 0) + ",";
        json += "\"vcu_sim\":" + String(data.vcuSimMode ? 1 : 0) + ",";
        json += "\"vcu_inv12v\":" + String(data.vcuInv12v ? 1 : 0) + ",";
        json += "\"vcu_hv\":" + String(data.vcuHvVoltage, 1) + ",";
        json += "\"vcu_torque_demand\":" + String(data.vcuTorqueDemandNm, 2) + ",";
        json += "\"vcu_rplus\":" + String(data.vcuRbPlusCmd ? 1 : 0) + ",";
        json += "\"vcu_precharge\":" + String(data.vcuPrechargeCmd ? 1 : 0) + ",";
        json += "\"vcu_ssr\":" + String(data.vcuSsrCmd ? 1 : 0) + ",";
        json += "\"vcu_rminus\":" + String(data.vcuRMinusCmd ? 1 : 0) + ",";
        {
            const char* vcuSt;
            if (!data.vcuInv12v)         vcuSt = "WAIT_12V";
            else if (data.vcuPrechargeCmd) vcuSt = "PRECHARGE";
            else if (data.vcuRbPlusCmd)    vcuSt = "ARMED";
            else                           vcuSt = "IDLE";
            json += "\"vcu_state\":\"";
            json += vcuSt;
            json += "\",";
        }
        json += "\"fw_state\":\"" + String(MetaSense::HardwareOutputStateMachine::stateName()) + "\",";
        json += "\"hw_rbplus\":" + String(MetaSense::HardwareOutputStateMachine::isRbPlusActive() ? 1 : 0) + ",";
        json += "\"hw_precharge\":" + String(MetaSense::HardwareOutputStateMachine::isPrechargeActive() ? 1 : 0) + ",";
        json += "\"hw_rbminus\":" + String(MetaSense::HardwareOutputStateMachine::isRbMinusActive() ? 1 : 0) + ",";
        json += "\"hw_ssr\":" + String(MetaSense::HardwareOutputStateMachine::isSsrActive() ? 1 : 0) + ",";
        json += "\"hw_prestart_warn\":" + String(MetaSense::HardwareOutputStateMachine::hasPrestartWarning() ? 1 : 0) + ",";

        if (floatChanged(data.throttlePercent, prevTelemetrySnapshot.throttlePercent, 0.1f)) {
            json += "\"throttle_pct\":" + String(data.throttlePercent, 0) + ",";
        }
        if (floatChanged(data.peakTorque, prevTelemetrySnapshot.peakTorque, 0.5f)) {
            json += "\"peakTorque\":" + String(data.peakTorque, 2) + ",";
        }
        if (floatChanged(data.peakTorque_RPM, prevTelemetrySnapshot.peakTorque_RPM, 5.0f)) {
            json += "\"peakTorque_RPM\":" + String(data.peakTorque_RPM, 0) + ",";
        }
    } else {
        // Trend path: send only fields used by trend live graphs on every frame.
        json += "\"kw\":" + String(data.kw, 2) + ",";
        json += "\"torque\":" + String(data.torqueNm, 2) + ",";
        json += "\"torque_measured\":" + String(measuredLoadTorqueNm, 2) + ",";
        json += "\"lambda\":" + String(data.lambdaValue, 3) + ",";
        json += "\"massflow_m3h\":" + String(data.massflowM3h, 1) + ",";
    }

    if (!trendMinimalTelemetry) {
        json += "\"recording\":" + String(isRecording ? 1 : 0) + ",";
        prevRecordingSent = isRecording;
        prevRecordingSentInitialized = true;
    }

    const bool emitSlowTelemetry = (gTelemetryProfile == TelemetryProfile::Full) && !trendMinimalTelemetry;

    // Slow telemetry fields are sent as evenly sized micro-chunks
    // (one field per fast frame) to minimize bursty UI workload.
    // Step-1 scaffold: profile defaults to Full, so behavior remains unchanged.
    if (emitSlowTelemetry) {
        switch (slowSliceNow) {
            case 0:
                json += "\"air_density\":" + String(data.airDensity, 3) + ",";
                break;
            case 1:
                json += "\"climate_cf\":" + String(data.climateCF, 4) + ",";
                break;
            case 2:
                json += "\"ambient_temp\":" + String(data.ambientC, 1) + ",";
                break;
            case 3:
                json += "\"pressure\":" + String(data.pressureHpa, 1) + ",";
                break;
            case 4:
                json += "\"egt_hot\":" + String(data.egtHotC, 1) + ",";
                break;
            case 5:
                json += "\"rel_humidity\":" + String(data.humidity, 1) + ",";
                break;
            case 6:
                json += "\"energy\":" + String(data.energyMJ, 2) + ",";
                break;
            case 7:
                json += "\"lambda\":" + String(data.lambdaValue, 3) + ",";
                break;
            case 8:
                json += "\"massflow_m3h\":" + String(data.massflowM3h, 1) + ",";
                break;
            case 9:
            {
                const bool energyActive = MetaSense::DynoStateMachine::isEnergyMeasuring();
    #if METASENSE_DASHBOARD_TRANSITIONAL_RUN_FIELDS
                if (!prevEnergyActiveInitialized || energyActive != prevEnergyActiveSent) {
                    json += "\"energy_active\":" + String(energyActive ? 1 : 0) + ",";
                    prevEnergyActiveSent = energyActive;
                    prevEnergyActiveInitialized = true;
                }
    #else
                json += "\"energy_active\":" + String(energyActive ? 1 : 0) + ",";
    #endif
                break;
            }
            case 10:
                json += "\"egt_status\":" + String(data.egtStatus) + ",";
                break;
            case 11:
                json += "\"egt_ready\":" + String(data.egtReady ? 1 : 0) + ",";
                break;
            case 12:
            {
                const uint8_t dynoModeNow = static_cast<uint8_t>(data.mode);
    #if METASENSE_DASHBOARD_TRANSITIONAL_RUN_FIELDS
                if (!prevDynoModeInitialized || dynoModeNow != prevDynoModeSent) {
                    json += "\"dyno_mode\":\"" + String(MetaSense::toString(data.mode)) + "\",";
                    prevDynoModeSent = dynoModeNow;
                    prevDynoModeInitialized = true;
                }
    #else
                json += "\"dyno_mode\":\"" + String(MetaSense::toString(data.mode)) + "\",";
    #endif
                break;
            }
            case 13:
    #if METASENSE_DASHBOARD_TRANSITIONAL_RUN_FIELDS
                if (!prevVcuReadyInitialized || data.vcuReady != prevVcuReadySent) {
                    json += "\"vcu_ready\":" + String(data.vcuReady ? 1 : 0) + ",";
                    prevVcuReadySent = data.vcuReady;
                    prevVcuReadyInitialized = true;
                }
    #else
                json += "\"vcu_ready\":" + String(data.vcuReady ? 1 : 0) + ",";
    #endif
                break;
            case 14:
            default:
    #if METASENSE_DASHBOARD_TRANSITIONAL_RUN_FIELDS
                if (!prevSwActiveInitialized || data.swActive != prevSwActiveSent) {
                    json += "\"sw_active\":" + String(data.swActive ? 1 : 0) + ",";
                    prevSwActiveSent = data.swActive;
                    prevSwActiveInitialized = true;
                }
    #else
                json += "\"sw_active\":" + String(data.swActive ? 1 : 0) + ",";
    #endif
                break;
        }
    }

    // Remove trailing comma and close JSON
    if (json.endsWith(",")) {
        json.remove(json.length() - 1);
    }
    json += "}";

#if METASENSE_JSON_DELAY_COUNTERS
    // Measure JSON payload size when delay counters enabled
    const uint16_t builtLen = static_cast<uint16_t>(json.length());
    wsJsonLastLen = builtLen;
    if (builtLen > wsJsonMaxLen) {
        wsJsonMaxLen = builtLen;
    }
    if (builtLen > 420) {  // kWebSocketJsonReserve hardcoded for measurement
        ++wsJsonOverReserveTotal;
    }
#endif

    // Update snapshot for next comparison
    prevTelemetrySnapshot = data;

    for (auto& client : wsock.getClients()) {
#if METASENSE_STREAM_DIAGNOSTICS
        ++wsClients;
#endif
        // Keep telemetry fresh: drop if queue has anything pending (stale frames)
        // With 50ms publish rate, this maintains tight latency while reducing WiFi saturation
        if (!client.canSend()) {
#if METASENSE_STREAM_DIAGNOSTICS
            ++wsSkipNoSendNow;
            ++wsSkipNoSendTotal;
#endif
            continue;
        }
        if (client.queueIsFull()) {
#if METASENSE_STREAM_DIAGNOSTICS
            ++wsSkipFullNow;
            ++wsSkipFullTotal;
#endif
            continue;
        }
        if (client.queueLen() > 0) {
#if METASENSE_STREAM_DIAGNOSTICS
            ++wsSkipBacklogNow;
            ++wsSkipBacklogTotal;
#endif
            continue;
        }
        client.text(json);
#if METASENSE_STREAM_DIAGNOSTICS
        ++wsSentNow;
        ++wsSentTotal;
#endif
    }

#if METASENSE_STREAM_DIAGNOSTICS
    (void)wsClients;
    (void)wsSentNow;
#endif
}

void publishTelemetry()
{
    static uint32_t lastPublishedVersion = 0;
    static uint32_t lastSendMs = 0;

    const uint32_t now = millis();
    if (now - lastSendMs < kWebSocketPublishPeriodMs) {
        return;
    }

    const uint32_t currentVersion = MetaSense::RunStorage::version();
    if (currentVersion == lastPublishedVersion) {
        return;
    }

    lastPublishedVersion = currentVersion;
    lastSendMs = now;
    const MetaSense::Telemetry telemetry = MetaSense::RunStorage::latest();
    notifyClients(telemetry, MetaSense::DynoStateMachine::isRecording());
}

void notifyRunComplete(const MetaSense::Telemetry& data)
{
    if (MetaSense::WebSocketServer::socket().count() == 0) {
        return;
    }

    String json;
    json.reserve(192);
    json = "{\"type\":\"run_complete\"";
    json += ",\"peakKW\":" + String(data.peakKW, 2);
    json += ",\"peakKW_RPM\":" + String(data.peakKW_RPM, 0);
    json += ",\"peakTorque\":" + String(data.peakTorque, 2);
    json += ",\"peakTorque_RPM\":" + String(data.peakTorque_RPM, 0);
    json += ",\"peakEGT\":" + String(data.egtHotC, 1);
    json += "}";
    MetaSense::WebSocketServer::socket().textAll(json);
}

void sendCalibrationInfo(const String& msg)
{
    MetaSense::WebSocketServer::socket().textAll("{\"type\":\"info\",\"msg\":\"" + msg + "\"}");
}

void calibrationTask(void* /*pvParameters*/)
{
    for (;;) {
        bool shouldRun = false;
        bool shouldTare = false;
        float knownWeightKg = 0.0f;

        portENTER_CRITICAL(&calibMux);
        if (!calibBusy && tareRequestPending) {
            calibBusy = true;
            tareRequestPending = false;
            shouldRun = true;
            shouldTare = true;
        } else if (calibRequestPending && !calibBusy) {
            calibBusy = true;
            calibRequestPending = false;
            knownWeightKg = calibKnownWeightKg;
            shouldRun = true;
        }
        portEXIT_CRITICAL(&calibMux);

        if (shouldRun) {
            if (shouldTare) {
                sendCalibrationInfo("Tare started...");
                MetaSense::Input::tare();
                MetaSense::RunStorage::saveCalibration();
                sendCalibrationInfo("Tare applied (zero=" + String(MetaSense::Input::getZeroOffsetRaw(), 2) + ")");
            } else {
                loadCellExclusiveSampling = true;
                setLoadCellStableSampling(true);
                sendCalibrationInfo("Calibrating... collecting samples for 5 seconds (tare first!)");
                vTaskDelay(pdMS_TO_TICKS(kCalibrationSettlingMs));

                float seedRaw = getLoadCellSamplerAverageRaw();
                if (!isfinite(seedRaw)) {
                    seedRaw = filteredAdc;
                }
                resetLoadRawAverage(seedRaw);
                resetLoadKgAverage(0.0f);

                float acceptedSamples[96];
                int validCount = 0;
                uint8_t acceptedCount = 0;
                const uint32_t startTime = millis();

                while ((millis() - startTime) < kCalibrationDurationMs) {
                    float rawSample = 0.0f;
                    if (readAveragedDirectLoadRawSample(rawSample, kLoadRawBurstSamples, 12)) {
                        const float smoothedRaw = applyLoadRawAverage(rawSample);
                        const float current = smoothedRaw - zeroOffset;
                        if (isfinite(current)) {
                            if (validCount >= kCalibrationSkipFirstSamples && acceptedCount < 96) {
                                acceptedSamples[acceptedCount++] = current;
                            }
                            ++validCount;
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(kCalibrationSamplePeriodMs));
                }

                if (acceptedCount == 0) {
                    sendCalibrationInfo("Calibration failed. No valid samples captured.");
                } else {
                    float avgRaw = 0.0f;
                    float maxDev = 0.0f;
                    const bool haveStats = computeTrimmedStats(acceptedSamples,
                                                               acceptedCount,
                                                               avgRaw,
                                                               maxDev);
                    if (!haveStats || !isfinite(avgRaw)) {
                        sendCalibrationInfo("Calibration failed. Invalid raw average.");
                    } else if (fabsf(avgRaw) < 1e-6f) {
                        sendCalibrationInfo("Calibration skipped: dummy-zero condition (factor unchanged).");
                    } else {
                        const float factor = knownWeightKg / avgRaw;
                        if (!isfinite(factor) || fabsf(factor) < 1e-12f) {
                            sendCalibrationInfo("Calibration failed. Invalid factor.");
                        } else {
                            MetaSense::Input::setCalibrationFactor(factor);
                            sendCalibrationInfo("Calibration done! New factor: " + String(factor, 6));
                        }
                    }
                }

                loadCellExclusiveSampling = false;
                setLoadCellStableSampling(false);
            }

            portENTER_CRITICAL(&calibMux);
            calibBusy = false;
            portEXIT_CRITICAL(&calibMux);
        }

        vTaskDelay(pdMS_TO_TICKS(kCalibrationSamplePeriodMs));
    }
}

} // anonymous namespace


namespace MetaSense::Input { // EXTERNAL SCOPE

void tareMainGui()
{
    (void)requestTare();
}

bool requestTare()
{
    bool accepted = false;
    portENTER_CRITICAL(&calibMux);
    if (!calibBusy && !calibRequestPending && !tareRequestPending) {
        tareRequestPending = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&calibMux);
    return accepted;
}

void tare()
{
    float avgRaw = 0.0f;
    float maxDev = 0.0f;
    setLoadCellStableSampling(true);
    delay(kCalibrationSettlingMs);
    if (sampleLoadRawStats(avgRaw, maxDev, 1000, 48)) {
        zeroOffset = avgRaw;
        // Bound deadband to avoid tare under unstable noise from muting torque output.
        zeroDeadbandRaw = min(kTareDeadbandMaxRaw, max(0.0f, maxDev * 3.0f));
        filteredAdc = avgRaw;
        resetLoadCellSampler(avgRaw);
        resetLoadRawAverage(avgRaw);
        resetLoadKgAverage(0.0f);
        loadKgFilt = 0.0f;
    } else {
        float fallbackRaw = getLoadCellSamplerAverageRaw();
        if (!isfinite(fallbackRaw)) {
            fallbackRaw = filteredAdc;
        }
        // Keep tare offset semantics aligned with runtime raw-filtered path.
        zeroOffset = applyLoadRawAverage(fallbackRaw);
        filteredAdc = zeroOffset;
        zeroDeadbandRaw = 0.0f;
        resetLoadCellSampler(filteredAdc);
        resetLoadRawAverage(filteredAdc);
        resetLoadKgAverage(0.0f);
        loadKgFilt = 0.0f;
    }
    setLoadCellStableSampling(false);
    // Force immediate zero output on the load-cell torque path after tare.
    torqueFilt = 0.0f;
    tele.loadKg = 0.0f;
    tele.torqueNm = 0.0f;
    tele.brakeTorqueNm = 0.0f;
}

void setCalibrationFactor(float factor)
{
    calibrationFactor = factor;
    MetaSense::RunStorage::saveCalibration();
}

uint16_t getLoadCellSampleRateSps()
{
    return getLoadCellSampleRateSpsLocal();
}

bool applyLoadCellSettingsProfile()
{
    if (!loadCellNauReady) {
        return false;
    }

    const bool prevExclusive = loadCellExclusiveSampling;
    loadCellExclusiveSampling = true;
    const bool ok = applyLoadCellNauProfile(currentRuntimeLoadCellGain(),
                                            currentRuntimeLoadCellRate(),
                                            "settings_apply");
    loadCellExclusiveSampling = prevExclusive;

    float seedRaw = getLoadCellSamplerAverageRaw();
    if (!isfinite(seedRaw)) {
        seedRaw = filteredAdc;
    }
    resetLoadCellSampler(seedRaw);
    return ok;
}

bool calibrateWithKnownWeight(float knownWeightKg, float& outFactor)
{
    if (!(knownWeightKg > 0.0f) || !isfinite(knownWeightKg)) {
        return false;
    }

    float avgRaw = 0.0f;
    float maxDev = 0.0f;
    loadCellExclusiveSampling = true;
    setLoadCellStableSampling(true);
    delay(kCalibrationSettlingMs);
    if (!sampleLoadRawStats(avgRaw, maxDev, 1000, 32)) {
        loadCellExclusiveSampling = false;
        setLoadCellStableSampling(false);
        return false;
    }
    loadCellExclusiveSampling = false;
    setLoadCellStableSampling(false);

    // Calibration uses current averaged raw sensor counts relative to tare offset.
    const float netRaw = avgRaw - zeroOffset;
    if (!isfinite(netRaw)) {
        return false;
    }

    if (fabsf(netRaw) < 1e-6f) {
        // Dummy-zero condition: keep current factor and report success to caller.
        outFactor = getCalibrationFactor();
        return true;
    }

    const float factor = knownWeightKg / netRaw;
    if (!isfinite(factor)) {
        return false;
    }

    setCalibrationFactor(factor);
    outFactor = factor;
    return true;
}

bool requestCalibrationWithKnownWeight(float knownWeightKg)
{
    if (!(knownWeightKg > 0.0f) || !isfinite(knownWeightKg)) {
        return false;
    }

    bool accepted = false;
    portENTER_CRITICAL(&calibMux);
    if (!calibBusy && !calibRequestPending) {
        calibKnownWeightKg = knownWeightKg;
        calibRequestPending = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&calibMux);
    return accepted;
}

float getCalibrationFactor()
{
    return calibrationFactor;
}

float getZeroOffset()
{
    return zeroOffset;
}

float getZeroOffsetRaw()
{
    return zeroOffset / kLoadCellRawScale;
}

float rpm()
{
    return tele.rpm;
}

float torqueNm()
{
    return tele.torqueNm;
}

float egtHotC()
{
    return tele.egtHotC;
}

void getEnvironment(float& ambientC,
                    float& pressureHpa,
                    float& humidityPct,
                    float& airDensity,
                    float& climateCf)
{
    ambientC = tele.ambientC;
    pressureHpa = tele.pressureHpa;
    humidityPct = tele.humidity;
    airDensity = tele.airDensity;
    climateCf = tele.climateCF;
}

float currentKpLive()
{
    return (MetaSense::Settings::usePot3Kp && lastAppliedKp >= 0.0f)
        ? lastAppliedKp
        : MetaSense::Settings::kp;
}

bool isVcuReady()
{
    return tele.vcuReady;
}

const char* vcuReadySource()
{
    const uint32_t now = millis();
    const LeafInvFeedback& fb = MetaSense::CANBus::feedback();
    const bool nativeStatusFresh = (fb.status_update_ms != 0U) &&
                                   (elapsedMsSafe(now, fb.status_update_ms) < CAN_TEMP_TIMEOUT_MS);
    const bool rpmFreshForFallback = (fb.rpm_update_ms != 0U) &&
                                     (elapsedMsSafe(now, fb.rpm_update_ms) <= CAN_RX_TARGET_MAX_AGE_MS);

    if (!tele.vcuReady) {
        return "not_ready";
    }

    if (nativeStatusFresh) {
        return "native_1d4";
    }

    if (kLeafCanVariantReadyFallback && rpmFreshForFallback) {
        return "fallback_rpm_1da";
    }

    if (rpmFreshForFallback) {
        return "rpm_frame_1da";
    }

    return "unknown";
}

void setUiModeHintTrend(bool trendMode)
{
    gUiModeHintTrend = trendMode;
}

bool isUiModeHintTrend()
{
    return gUiModeHintTrend;
}

void getLoadCellInitStatus(bool& ldoConfigured, bool& internalCalOk, uint8_t& internalCalAttempts)
{
    ldoConfigured = loadCellNauLdoConfigured;
    internalCalOk = loadCellNauInternalCalOk;
    internalCalAttempts = loadCellNauInternalCalAttempts;
}

void getLoadCellSamplerRuntime(uint32_t& lastUs, uint32_t& maxUs, uint32_t& emaUs, uint32_t& loops)
{
    lastUs = loadCellSamplerLastUs;
    maxUs = loadCellSamplerMaxUs;
    emaUs = loadCellSamplerEmaUs;
    loops = loadCellSamplerLoops;
}

void resetLoadCellSamplerMaxRuntime()
{
    loadCellSamplerMaxUs = 0;
}

void updateCanRpm(float rpm)
{
    const uint32_t now = millis();
    if (now - lastCanRpmUpdate < CAN_RPM_MIN_UPDATE_MS) {
        return;
    }

    if (fabs(rpm - lastCanRpm) > CAN_MAX_JUMP)
        return;

    if (rpm > 0 && rpm < 20000) {
        canRpm = rpm;
        lastCanRpm = rpm;
        lastCanRpmUpdate = now;
        lastCanLeafAnyUpdate = now;
    }
}

void updateCanTorque(float torqueNm)
{
    (void)torqueNm;
    // CAN torque input is intentionally not consumed on this project profile.
}

void updateCanTemps(float inverterTempC, float statorTempC, float coolantTempC)
{
    if (isfinite(inverterTempC)) {
        canInvTempC = inverterTempC;
        MetaSense::Inverter::setTemperatureC(inverterTempC);
    }
    if (isfinite(statorTempC)) {
        canStatorTempC = statorTempC;
    }
    if (isfinite(coolantTempC)) {
        canCoolantTempC = coolantTempC;
    }
    lastCanTempUpdate = millis();
    lastCanLeafAnyUpdate = lastCanTempUpdate;
}

void updateCanStatus(bool ready, bool fault, bool warning, bool limp)
{
    canInvReady = ready;
    canInvFault = fault;
    canInvWarning = warning;
    canInvLimp = limp;
    lastCanStatusUpdate = millis();
    lastCanLeafAnyUpdate = lastCanStatusUpdate;
}

bool sendLeafTorqueCommand55B(float torqueDemandNm,
                              bool readyBit,
                              bool hvOkBit,
                              bool brakeBit,
                              bool gearDriveBit)
{
    (void)brakeBit;
    const float torqueClamped = constrain(torqueDemandNm, -300.0f, 300.0f);
    const int16_t torqueRaw = static_cast<int16_t>(lroundf(torqueClamped * 10.0f));

    static uint8_t rollingCounter = 0;

    uint8_t data[8] = {0};
    data[0] = static_cast<uint8_t>(torqueRaw & 0xFF);
    data[1] = static_cast<uint8_t>((torqueRaw >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((readyBit ? 0x01 : 0x00) |
                                   (hvOkBit ? 0x02 : 0x00) |
                                   0x00 |
                                   (gearDriveBit ? 0x08 : 0x00));
    data[3] = static_cast<uint8_t>(rollingCounter & 0x0F);
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;

    // Leaf VCM bringup checklist checksum:
    // sum(Byte0 + Byte1 + Byte2 + Byte3 + Byte4 + Byte6) & 0xFF
    const uint16_t checksum = static_cast<uint16_t>(data[0]) +
                              static_cast<uint16_t>(data[1]) +
                              static_cast<uint16_t>(data[2]) +
                              static_cast<uint16_t>(data[3]) +
                              static_cast<uint16_t>(data[4]) +
                              static_cast<uint16_t>(data[6]);
    data[7] = static_cast<uint8_t>(checksum & 0xFF);

    rollingCounter = static_cast<uint8_t>((rollingCounter + 1) & 0x0F);
    const bool sent = MetaSense::CANBus::send(0x55B, data, 8);
    if (sent) {
        const uint32_t nowMs = millis();
        const float previousCmd = s_leafLastSentTorqueNm;
        s_leafLastSentTorqueNm = torqueClamped;
        s_leafLastSentTorqueMs = nowMs;

        if (fabsf(torqueClamped - previousCmd) >= kLeafTorqueTrackStepNm) {
            s_leafTorqueTrackStartMs = nowMs;
            s_leafTorqueTrackTargetNm = torqueClamped;
            s_leafTorqueTrackPending = true;
        }
    }

    return sent;
}

void updateVcuDebug(bool simMode,
                    bool inv12v,
                    float hvVoltage,
                    float torqueDemandNm,
                    bool rPlus,
                    bool precharge,
                    bool ssr,
                    bool rMinus)
{
    vcuDebugSimMode = simMode;
    vcuDebugInv12v = inv12v;
    vcuDebugHvVoltage = isfinite(hvVoltage) ? hvVoltage : 0.0f;
    vcuDebugTorqueDemandNm = isfinite(torqueDemandNm) ? torqueDemandNm : 0.0f;
    vcuDebugRPlus = rPlus;
    vcuDebugPrecharge = precharge;
    vcuDebugSsr = ssr;
    vcuDebugRMinus = rMinus;
}

void begin()
{
    tele = MetaSense::Telemetry();
    MetaSense::CANBus::reset();
    s_leafVcmState = LeafVcmBringupState::CanOnline;
    s_leafVcmStateSinceMs = millis();
    s_leafTxGapFault = false;
    s_leafVcmFaultLatched = false;
    s_leafVcmLastFaultMs = 0;
    s_leafCanPartnerSeen = false;
    s_leafSimFeedbackActive = false;
    s_leafSimLastInjectMs = 0;
    s_leafSimLogPrinted = false;
    s_leafListenOnlyLogPrinted = false;
    s_leafHandshakeSent = false;
    s_leafHandshakeArmed = false;
    s_leafHandshakeSentCount = 0;
    s_leafHandshakeAttemptCount = 0;
    s_leafHandshakeStartMs = 0;
    s_leafHandshakeLastAttemptMs = 0;
    s_leafHandshakePromotedLogPrinted = false;
    s_leafLastSentTorqueNm = 0.0f;
    s_leafLastSentTorqueMs = 0;
    s_leafTorqueTrackStartMs = 0;
    s_leafTorqueTrackLatencyMs = 0;
    s_leafTorqueTrackPending = false;
    s_leafTorqueTrackTargetNm = 0.0f;
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
    s_leafPreStatusLastMs = 0;
#endif
    s_leafRxDiagLastMs = 0;
    s_leafRxWarnLastMs = 0;
    s_leafRxAwaitPartnerLastMs = 0;
    s_leafTxGapTestCycleStartMs = 0;
    s_leafTxGapTestActive = false;
    s_leafTxGapTestLoggedStart = false;
    s_leafTxGapTestLoggedEnd = false;
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
    lastCanAltDiagMs = 0;
    lastCanAlt120FramesLogged = 0;
    lastCanAlt55aFramesLogged = 0;
#endif
    lastCanBusOffSeen = 0;
    lastCanStatusQueryFailuresSeen = 0;
    lastCanRpmFrameMs = 0;
    lastCanTorqueFrameMs = 0;
    lastCanTempsFrameMs = 0;
    lastCanStatusFrameMs = 0;
    zeroOffset = 0.0f;
    calibrationFactor = 0.01f;
    filteredAdc = 0.0f;
    resetLoadRawAverage(filteredAdc);
    resetLoadKgAverage(0.0f);
    resetTachoRawAverage(0.0f);
    resetMassflowRawAverage(0.0f);
    resetLambdaRawAverage(1.0f);
    {
        float storedZero = 0.0f;
        float storedFactor = 0.01f;
        if (MetaSense::RunStorage::loadCalibration(storedZero, storedFactor)) {
            if (isfinite(storedZero)) {
                zeroOffset = storedZero;
                filteredAdc = storedZero;
                resetLoadCellSampler(filteredAdc);
                resetLoadRawAverage(filteredAdc);
                resetLoadKgAverage(0.0f);
            }
            if (isfinite(storedFactor) && storedFactor > 0.0f) {
                calibrationFactor = storedFactor;
            }
        }
    }
    kpPotFilteredAdc = -1.0f;
    lastAppliedKp = -1.0f;
    if (loadCellMutex == nullptr) {
        loadCellMutex = xSemaphoreCreateMutex();
    }
    resetLoadCellSampler(filteredAdc);
    MetaSense::ControlTask::configurePI(MetaSense::Settings::kp,
                                        MetaSense::Settings::ki,
                                        TORQUE_MIN,
                                        TORQUE_MAX);
    if (MetaSense::Settings::usePot3Kp) {
        applyRuntimeKpFromPot(true);
    }

    // Digital inputs
    pinMode(MetaSense::Globals::kRampSwitchPin, INPUT_PULLDOWN);  // SW switch – active HIGH
    pinMode(MetaSense::Globals::kRbPlusInputPin, INPUT_PULLDOWN); // VCU ready – active HIGH
    prevSwState = (digitalRead(MetaSense::Globals::kRampSwitchPin) == HIGH);
    const int vcuRbPlusLevel = digitalRead(MetaSense::Globals::kRbPlusInputPin);
    Serial.printf("[Input] VCU ready source: CAN_INVERTER_STATUS (legacy_cfg_gpio=%d, RB+=%d)\n",
                  MetaSense::Globals::kVcuSwitch ? 1 : 0,
                  vcuRbPlusLevel);
    Serial0.printf("[Input] VCU ready source: CAN_INVERTER_STATUS (legacy_cfg_gpio=%d, RB+=%d)\n",
                   MetaSense::Globals::kVcuSwitch ? 1 : 0,
                   vcuRbPlusLevel);

    // Bring relay/control outputs to a known-safe state before probing I2C sensors.
    MetaSense::HardwareOutputStateMachine::begin();

    if (kEgtOnlyI2cInitMode || kNauOnlyI2cInitMode) {
        ambientBmeReady = false;
        ambientBmeReadPending = false;
        Serial.println("[Input] NAU-only I2C mode: skipping BME680 initialization");
        Serial0.println("[Input] NAU-only I2C mode: skipping BME680 initialization");
    } else {
        ambientBmeReady = ambientBme.begin(0x76) || ambientBme.begin(0x77);
        ambientBmeReadPending = false;
        if (ambientBmeReady) {
            ambientBme.setTemperatureOversampling(BME680_OS_1X);
            ambientBme.setHumidityOversampling(BME680_OS_1X);
            ambientBme.setPressureOversampling(BME680_OS_1X);
            ambientBme.setIIRFilterSize(BME680_FILTER_SIZE_0);
            ambientBme.setGasHeater(0, 0);
            Serial.println("[Input] Ambient source ready (BME680)");
            Serial0.println("[Input] Ambient source ready (BME680)");
        } else {
            Serial.println("[Input] Ambient source unavailable (BME680)");
            Serial0.println("[Input] Ambient source unavailable (BME680)");
        }
    }

    if (!kNauOnlyI2cInitMode) {
        delay(kMcp9600BootSettleDelayMs);
        egtDigitalReady = false;
        for (uint8_t attempt = 1; attempt <= 3; ++attempt) {
            egtDigitalReady = egtDigital.begin();
            if (egtDigitalReady) {
                break;
            }
            if (attempt < 3) {
                delay(kMcp9600BootRetryDelayMs);
            }
        }
        if (egtDigitalReady) {
            Serial.println("[Input] EGT digital source ready (MCP9600)");
            Serial0.println("[Input] EGT digital source ready (MCP9600)");
        } else {
            Serial.println("[Input] EGT digital source unavailable after 3 boot attempts");
            Serial0.println("[Input] EGT digital source unavailable after 3 boot attempts");
        }
    } else {
        egtDigitalReady = false;
        Serial.println("[Input] NAU-only I2C mode: skipping MCP9600 initialization");
        Serial0.println("[Input] NAU-only I2C mode: skipping MCP9600 initialization");
    }

    tryInitLoadCellNau();

    if (loadCellSamplerTaskHandle == nullptr) {
        BaseType_t samplerCreated = xTaskCreatePinnedToCore(
            loadCellSamplerTask,
            "loadCellSampler",
            4096,
            nullptr,
            2,
            &loadCellSamplerTaskHandle,
            0);
        if (samplerCreated != pdPASS) {
            Serial.println("[Input] Failed to start load-cell sampler task");
            Serial0.println("[Input] Failed to start load-cell sampler task");
        }
    }

    updateI2cScanSummary();
    Serial.printf("[Input] I2C devices: %s\n", i2cScanSummary.c_str());
    Serial0.printf("[Input] I2C devices: %s\n", i2cScanSummary.c_str());

    updateAmbientInputs(true);

    if (calibrationTaskHandle == nullptr) {
        BaseType_t created = xTaskCreatePinnedToCore(
            calibrationTask,
            "calibrationTask",
            4096,
            nullptr,
            1,
            &calibrationTaskHandle,
            0);
        if (created != pdPASS) {
            Serial.println("[Input] Failed to start calibration task");
            Serial0.println("[Input] Failed to start calibration task");
        }
    }
}

void startRecording()
{
    tele.recording = true;
    resetFallbackRunLog(millis());

    tele.peakTorque     = 0.0f;
    tele.peakTorque_RPM = 0.0f;
    tele.peakKW         = 0.0f;
    tele.peakKW_RPM     = 0.0f;

    tele.maxRpm         = 0.0f;
    tele.maxTorqueNm    = 0.0f;

    MetaSense::ControlTask::resetPI();
}

void stopRecording()
{
    tele.recording = false;
    MetaSense::HardwareOutputStateMachine::stop();
}

void loop()
{
    static bool prevRecording = false;
    static uint32_t last = millis();
    static uint32_t lastEgtRetryMs = 0;
    uint32_t now = millis();
    float dtSec = (now - last) / 1000.0f;
    last = now;

    if (!loadCellNauReady && (now - lastLoadCellNauRetryMs) >= kLoadCellNauRetryPeriodMs) {
        lastLoadCellNauRetryMs = now;
        if (!loadCellExclusiveSampling) {
            tryInitLoadCellNau();
            updateI2cScanSummary();
        }
    }

    if (!loadCellExclusiveSampling && !egtDigital.isReady() && (now - lastEgtRetryMs) >= 10000) {
        lastEgtRetryMs = now;
        egtDigitalReady = egtDigital.begin();
    }

    pollLeafCanFrames(now);
    maybeInjectLeafSimFeedback(now);

    if (kLeafCanVariantReadyFallback) {
        const LeafInvFeedback& leafFbStatus = MetaSense::CANBus::feedback();
        const bool nativeStatusFresh = (leafFbStatus.status_update_ms != 0U) &&
                                       (elapsedMsSafe(now, leafFbStatus.status_update_ms) < CAN_TEMP_TIMEOUT_MS);
        const bool rpmFreshForFallback = (lastCanRpmFrameMs != 0U) &&
                                         (elapsedMsSafe(now, lastCanRpmFrameMs) <= CAN_RX_TARGET_MAX_AGE_MS);

        // Variant fallback: when 0x1D4 status is missing, derive readiness from
        // 0x1DA (RPM) freshness only.
        if (!nativeStatusFresh && rpmFreshForFallback) {
            canInvReady = true;
            canInvFault = false;
            canInvWarning = (leafFbStatus.mg_error_codes != 0U);
            canInvLimp = false;
            lastCanStatusUpdate = now;
            lastCanLeafAnyUpdate = now;
        }
    }

    if (MetaSense::Settings::usePot3Kp) {
        applyRuntimeKpFromPot(false);
    }

    const float manualRpmTarget = readRpmSetpointPot();

    // VCU ready for bring-up is driven by inverter status response on CAN.
    // Keep OFF until fresh status indicates inverter-ready and no active fault/limp.
    const bool inverterStatusFresh = (lastCanStatusUpdate != 0U) &&
                                     (elapsedMsSafe(now, lastCanStatusUpdate) < CAN_TEMP_TIMEOUT_MS);
    const bool vcuReadyBase = inverterStatusFresh && canInvReady && !canInvFault && !canInvLimp;
    tele.vcuReady = vcuReadyBase;

    // --- SW switch recording toggle (GPIO 35, active HIGH, debounced) ---
    {
        const bool swNow = (digitalRead(MetaSense::Globals::kRampSwitchPin) == HIGH);
        if (swNow != prevSwState) {
            if ((now - swDebounceMs) >= kSwDebounceThresholdMs) {
                swDebounceMs = now;
                prevSwState = swNow;
                tele.swActive = swNow;
                if (swNow) {
                    // Rising edge: toggle recording
                    if (!MetaSense::DynoStateMachine::isRecording()) {
                        MetaSense::DynoStateMachine::startRecording();
                    } else {
                        MetaSense::DynoStateMachine::stopRecording();
                    }
                }
            }
        } else {
            swDebounceMs = now;
            tele.swActive = swNow;
        }
    }

    float alpha = MetaSense::Settings::filterAlpha;
    if (!isfinite(alpha)) {
        alpha = 0.2f;
    }
    alpha = constrain(alpha, 0.01f, 1.0f);

    // RPM source strategy:
    // - CAN mode: raw CAN RPM is e-motor RPM, test-engine RPM = e-motor RPM * gear ratio.
    // - Tachogen mode/fallback: tachogen is treated as test-engine RPM directly.
    tachoRpm = readTachoRpm();
    const bool canRpmAllowed = (!METASENSE_FORCE_TACHO_RPM_SOURCE) && MetaSense::Settings::useCanLeafRpm;
    bool canValid = canRpmAllowed && ((millis() - lastCanRpmUpdate) < CAN_RPM_TIMEOUT_MS);
    float rpmRaw = 0.0f;
    canFallbackActive = canRpmAllowed && !canValid;
    activeRpmFromCan = canValid;
    const float emotorRpmRaw = canRpm;
    const float rpmRatio = (MetaSense::Settings::virtGearRatio > 0.01f)
        ? MetaSense::Settings::virtGearRatio
        : 1.0f;
    const float canEngineRpm = emotorRpmRaw * rpmRatio;
    rpmRaw = canValid ? canEngineRpm : tachoRpm;

    rpmFilt = lpFilter(rpmFilt, rpmRaw, alpha);
    tele.rpm = rpmFilt;
    const bool canRpmMonitorFresh = (lastCanRpmMonitorUpdate != 0U) &&
                                    (elapsedMsSafe(now, lastCanRpmMonitorUpdate) < CAN_TEMP_TIMEOUT_MS);
    tele.leaf_rpm = canRpmMonitorFresh ? leafCanRpmMonitor : 0.0f;
    const LeafInvFeedback& leafFbNativeStatus = MetaSense::CANBus::feedback();
    const bool canTempsFresh = (lastCanTempUpdate != 0U) &&
                               (elapsedMsSafe(now, lastCanTempUpdate) < CAN_TEMP_TIMEOUT_MS);
    const bool canStatusFresh = (lastCanStatusUpdate != 0U) &&
                                (elapsedMsSafe(now, lastCanStatusUpdate) < CAN_TEMP_TIMEOUT_MS);
    const bool nativeCanStatusFresh = (leafFbNativeStatus.status_update_ms != 0U) &&
                                      (elapsedMsSafe(now, leafFbNativeStatus.status_update_ms) < CAN_TEMP_TIMEOUT_MS);
    const bool nativeInvReady = nativeCanStatusFresh ? leafFbNativeStatus.ready : false;
    const bool nativeInvFault = nativeCanStatusFresh &&
                                (leafFbNativeStatus.fault || leafFbNativeStatus.limp);
    const bool canTorqueUsable = (leafFbNativeStatus.torque_update_ms != 0U) &&
                                 (elapsedMsSafe(now, leafFbNativeStatus.torque_update_ms) < CAN_TEMP_TIMEOUT_MS);
    tele.leaf_torqueNm = canTorqueUsable
        ? leafFbNativeStatus.torque_nm
        : 0.0f;
    const bool leafCmdFresh = (s_leafLastSentTorqueMs != 0U) &&
                              (elapsedMsSafe(now, s_leafLastSentTorqueMs) < CAN_TEMP_TIMEOUT_MS);
    const float leafDemandNm = leafCmdFresh ? s_leafLastSentTorqueNm : 0.0f;
    tele.leaf_torqueDemandNm = leafDemandNm;
    const float leafTorqueErrorNm = (leafCmdFresh && canTorqueUsable)
        ? (leafDemandNm - tele.leaf_torqueNm)
        : 0.0f;
    tele.leaf_torqueErrorNm = leafTorqueErrorNm;
    if (leafCmdFresh && canTorqueUsable && fabsf(leafDemandNm) > 1.0f) {
        tele.leaf_torqueErrorPct = (leafTorqueErrorNm / leafDemandNm) * 100.0f;
    } else {
        tele.leaf_torqueErrorPct = 0.0f;
    }

    uint32_t elapsedTrackMs = 0U;
    if (s_leafTorqueTrackPending) {
        elapsedTrackMs = elapsedMsSafe(now, s_leafTorqueTrackStartMs);
        const float settleTolNm = max(kLeafTorqueTrackAbsTolNm,
                                      fabsf(s_leafTorqueTrackTargetNm) * kLeafTorqueTrackRelTol);
        if (canTorqueUsable && fabsf(tele.leaf_torqueNm - s_leafTorqueTrackTargetNm) <= settleTolNm) {
            s_leafTorqueTrackLatencyMs = static_cast<uint16_t>(min<uint32_t>(elapsedTrackMs, 65535U));
            s_leafTorqueTrackPending = false;
        } else if (elapsedTrackMs > kLeafTorqueTrackTimeoutMs) {
            s_leafTorqueTrackPending = false;
        }
    }
    tele.leaf_torqueTrackMs = s_leafTorqueTrackPending
        ? static_cast<uint16_t>(min<uint32_t>(elapsedTrackMs, 65535U))
        : s_leafTorqueTrackLatencyMs;
    tele.leaf_torqueTrackPending = s_leafTorqueTrackPending;
    tele.leaf_invTempC = canTempsFresh ? canInvTempC : 0.0f;
    tele.leaf_statorTempC = canTempsFresh ? canStatorTempC : 0.0f;
    tele.leaf_coolantTempC = canTempsFresh ? canCoolantTempC : 0.0f;
    tele.leaf_invReady = canStatusFresh ? canInvReady : false;
    tele.leaf_invFault = canStatusFresh ? canInvFault : false;
    tele.leaf_invWarning = canStatusFresh ? canInvWarning : false;
    tele.leaf_invLimp = canStatusFresh ? canInvLimp : false;
    tele.leaf_lastUpdateMs = lastCanLeafAnyUpdate;
    tele.vcuSimMode = vcuDebugSimMode;
    tele.vcuInv12v = vcuDebugInv12v;
    const bool canHvFresh = (leafFbNativeStatus.rpm_update_ms != 0U) &&
                            (elapsedMsSafe(now, leafFbNativeStatus.rpm_update_ms) < CAN_TEMP_TIMEOUT_MS);
    tele.vcuHvVoltage = canHvFresh ? leafFbNativeStatus.input_voltage : vcuDebugHvVoltage;
    tele.vcuTorqueDemandNm = vcuDebugTorqueDemandNm;
    tele.vcuRbPlusCmd = vcuDebugRPlus;
    tele.vcuPrechargeCmd = vcuDebugPrecharge;
    tele.vcuSsrCmd = vcuDebugSsr;
    tele.vcuRMinusCmd = vcuDebugRMinus;

    if (canRpmAllowed && canValid) {
        float delta = fabs(canEngineRpm - tachoRpm);
        rpmDeltaError = (delta > RPM_DELTA_LIMIT);
    } else {
        rpmDeltaError = false;
    }

    // other sensors
    float drumRaw = (canRpmAllowed && canValid) ? emotorRpmRaw : readDrumRpm();
    float loadRaw = readLoadKg();
    MetaSense::RunStorage::appendFsLiveProbeSample(static_cast<uint64_t>(esp_timer_get_time()), loadRaw);
    const bool captureActive = MetaSense::RunStorage::rawCaptureActive();
    const uint64_t nowUs = captureActive ? static_cast<uint64_t>(esp_timer_get_time()) : 0ULL;
    const uint64_t lastUs = lastRawCaptureAppendUs;
    const bool shouldAppendCapture = captureActive && (nowUs > (lastUs + 250000ULL));
    const float captureRawZeroed = loadRaw - zeroOffset;
    loadRaw = applyLoadRawAverage(loadRaw);
    const float captureFilteredZeroed = loadRaw - zeroOffset;
    if (shouldAppendCapture) {
        MetaSense::RunStorage::appendRawCaptureSample(nowUs,
                                                      captureRawZeroed,
                                                      captureFilteredZeroed,
                                                      loadCellNauReady ? "loop_guard" : "loop_fallback");
        lastRawCaptureAppendUs = nowUs;
    }

    drumRpmFilt = lpFilter(drumRpmFilt, drumRaw, alpha);
    filteredAdc = loadRaw;

    tele.drumRpm = drumRpmFilt;
    float netRaw = filteredAdc - zeroOffset;
    if (fabsf(netRaw) <= zeroDeadbandRaw) {
        netRaw = 0.0f;
    }
    const float loadKgRaw = netRaw * calibrationFactor;
    if (MetaSense::Settings::loadFilterMode == kLoadFilterModeTwoStageMovingAverage) {
        loadKgFilt = applyLoadKgMovingAverage(loadKgRaw);
    } else {
        loadKgFilt = lpFilter(loadKgFilt, loadKgRaw, alpha);
    }
    tele.loadKg  = loadKgFilt;

    if (!loadCellExclusiveSampling) {
        updateAmbientInputs(false);
        tele.egtReady    = egtDigital.isReady();
        tele.egtStatus   = egtDigital.status();
        tele.egtAddress  = egtDigital.address();
        tele.egtAckAddress = egtDigital.ackAddress();
        tele.egtHotC     = readEgtHotC();
        tele.egtAmbientC = readEgtAmbientC();
        tele.ambientC    = readAmbientC();
        tele.pressureHpa = readPressureHpa();
        tele.airDensity  = readAirDensity();
        tele.humidity    = readHumidity();
    }
    tele.massflowM3h = readMassflowM3h();
    tele.lambdaValue = readLambdaValue();

    const uint64_t captureNowUs = static_cast<uint64_t>(esp_timer_get_time());
    MetaSense::RunStorage::tickRawCapture(captureNowUs);
    MetaSense::RunStorage::tickFsLiveProbe(captureNowUs);

    MetaSense::DynoStateMachine::setManualRpmTarget(manualRpmTarget);
    MetaSense::DynoStateMachine::update();

    tele.mode = MetaSense::Settings::inertiaMode
        ? MetaSense::DynoMode::Inertia
        : MetaSense::DynoMode::Brake;

    if (MetaSense::DynoStateMachine::isAutoRunActive()) {
        // In autorun, preserve target provided by state machine path.
        tele.rpmTarget = MetaSense::Settings::getRpmTarget();
    } else {
        // In manual mode, pot is the active RPM target source.
        tele.rpmTarget = manualRpmTarget;
    }

    const float motorModeCapRpm = (MetaSense::Settings::motorModeMaxRpm > 0.0f)
        ? MetaSense::Settings::motorModeMaxRpm
        : RPM_SETPOINT_MAX;
    if (MetaSense::HardwareOutputStateMachine::isMotorState() && tele.rpmTarget > motorModeCapRpm) {
        tele.rpmTarget = motorModeCapRpm;
    }
    MetaSense::Settings::setRpmTarget(tele.rpmTarget);

    updateDyno(tele, dtSec);

    bool safe = checkSafety(tele);
    if (!safe) {
        tele.torqueNm      = 0.0f;
        tele.brakeTorqueNm = 0.0f;
    }

    float torqueCmd = MetaSense::ControlTask::computeTorqueCommand(
        tele.rpmTarget,
        tele.rpm,
        dtSec);
    if (!safe) torqueCmd = 0.0f;

    if (!leafCmdFresh) {
        // Keep UI demand visible even if 0x55B TX path is stale/offline.
        tele.leaf_torqueDemandNm = torqueCmd;
    }

#if METASENSE_TEST_TORQUE_OVERRIDE_ENABLED
    // Bench/test override: force fixed command whenever primary safety checks pass.
    if (safe) {
        torqueCmd = constrain(METASENSE_TEST_TORQUE_OVERRIDE_NM, -300.0f, 300.0f);
    } else {
        torqueCmd = 0.0f;
    }
#endif

    // eTorque is the PI torque demand signal.
    tele.eTorque = torqueCmd;
    // Keep VCU torque-demand telemetry aligned with the effective command path.
    tele.vcuTorqueDemandNm = torqueCmd;

    // Diagnostic visibility: show commanded torque directly in Leaf demand/error fields.
    tele.leaf_torqueDemandNm = torqueCmd;
    const float leafActualTorqueNm = 0.0f;
    tele.leaf_torqueNm = leafActualTorqueNm;
    tele.leaf_torqueErrorNm = 0.0f;
    tele.leaf_torqueErrorPct = 0.0f;

    const float maxAllowedRpm = (MetaSense::Settings::maxRPM > 0.0f) ? MetaSense::Settings::maxRPM : RPM_MAX_LIMIT;
    const float maxAllowedTorque = (MetaSense::Settings::maxTorque > 0.0f) ? MetaSense::Settings::maxTorque : TORQUE_MAX;
    const bool rpmLimitExceeded = tele.rpm > maxAllowedRpm;
    const bool torqueLimitExceeded = fabsf(tele.torqueNm) > maxAllowedTorque;
    const bool throttleSafetyCut = rpmLimitExceeded || torqueLimitExceeded;

    const float primaryBrakeSignedPercent = (torqueCmd / TORQUE_MAX) * 100.0f;
    // POT2 on AD1 (GPIO2, 0-3.3V) is the engine throttle setpoint source.
    // Map directly to GPIO47 PWM duty as 0-100% (0-3.3V equivalent after filtering).
    const float engineThrottlePercent = throttleSafetyCut ? 0.0f : readThrottlePotPercent();
    tele.throttlePercent = engineThrottlePercent;

    const bool relayInverterStatusReady = tele.vcuReady;
    const bool relayInverterReady = tele.vcuReady;
    const bool relayInverterFault = tele.leaf_invFault || tele.leaf_invLimp;

    MetaSense::HardwareOutputStateMachine::update(
        engineThrottlePercent,
        tele.rpmTarget,
        tele.rpm,
        primaryBrakeSignedPercent,
        tele.vcuHvVoltage,
        relayInverterStatusReady,
        relayInverterReady,
        relayInverterFault);

    const bool ssrActiveForLeafTx = MetaSense::HardwareOutputStateMachine::isSsrActive();
    const bool prechargeActiveForLeafTx = MetaSense::HardwareOutputStateMachine::isPrechargeActive();
    const bool prechargeSucceededForLeafTx = MetaSense::HardwareOutputStateMachine::isPrechargeSucceeded();
    const LeafInvFeedback& leafFbDiag = MetaSense::CANBus::feedback();
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
    if (s_leafPreStatusLastMs == 0U || (now - s_leafPreStatusLastMs) >= CAN_PRE_DIAG_LOG_PERIOD_MS) {
        const MetaSense::CANBus::Stats& canStatsDiag = MetaSense::CANBus::stats();
        const unsigned long feedbackAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, lastCanLeafAnyUpdate));
        const unsigned long id1daAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.rpm_update_ms));
        const unsigned long id1dbAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.torque_update_ms));
        const unsigned long id1dcAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.temps_update_ms));
        const unsigned long id1d4AgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.status_update_ms));
        Serial.printf("[VCM-PRE] state=%s can_partner=%d can_ready=%d ssr=%d precharge=%d precharge_ok=%d vcu_ready=%d inv_ready=%d inv_fault=%d inv_warn=%d inv_limp=%d hv=%.1f feedback_ms=%lu id1DA=%lu(age=%lu) id1DB=%lu(age=%lu) id1DC=%lu(age=%lu) id1D4=%lu(age=%lu) rx=%lu leaf_rx=%lu unk_rx=%lu tx=%lu last_id=0x%03lX\n",
                      leafVcmStateName(s_leafVcmState),
                      s_leafCanPartnerSeen ? 1 : 0,
                      MetaSense::CANBus::isReady() ? 1 : 0,
                      ssrActiveForLeafTx ? 1 : 0,
                      prechargeActiveForLeafTx ? 1 : 0,
                      prechargeSucceededForLeafTx ? 1 : 0,
                      tele.vcuReady ? 1 : 0,
                      tele.leaf_invReady ? 1 : 0,
                      tele.leaf_invFault ? 1 : 0,
                      tele.leaf_invWarning ? 1 : 0,
                      tele.leaf_invLimp ? 1 : 0,
                      tele.vcuHvVoltage,
                      feedbackAgeMs,
                      static_cast<unsigned long>(leafFbDiag.rpm_frames),
                      id1daAgeMs,
                      static_cast<unsigned long>(leafFbDiag.torque_frames),
                      id1dbAgeMs,
                      static_cast<unsigned long>(leafFbDiag.temps_frames),
                      id1dcAgeMs,
                      static_cast<unsigned long>(leafFbDiag.status_frames),
                      id1d4AgeMs,
                      static_cast<unsigned long>(canStatsDiag.rxFrames),
                      static_cast<unsigned long>(canStatsDiag.rxLeafFrames),
                      static_cast<unsigned long>(canStatsDiag.rxUnknownFrames),
                      static_cast<unsigned long>(canStatsDiag.txFrames),
                      static_cast<unsigned long>(canStatsDiag.lastRxId));
        Serial0.printf("[VCM-PRE] state=%s can_partner=%d can_ready=%d ssr=%d precharge=%d precharge_ok=%d vcu_ready=%d inv_ready=%d inv_fault=%d inv_warn=%d inv_limp=%d hv=%.1f feedback_ms=%lu id1DA=%lu(age=%lu) id1DB=%lu(age=%lu) id1DC=%lu(age=%lu) id1D4=%lu(age=%lu) rx=%lu leaf_rx=%lu unk_rx=%lu tx=%lu last_id=0x%03lX\n",
                       leafVcmStateName(s_leafVcmState),
                       s_leafCanPartnerSeen ? 1 : 0,
                       MetaSense::CANBus::isReady() ? 1 : 0,
                       ssrActiveForLeafTx ? 1 : 0,
                       prechargeActiveForLeafTx ? 1 : 0,
                       prechargeSucceededForLeafTx ? 1 : 0,
                       tele.vcuReady ? 1 : 0,
                       tele.leaf_invReady ? 1 : 0,
                       tele.leaf_invFault ? 1 : 0,
                       tele.leaf_invWarning ? 1 : 0,
                       tele.leaf_invLimp ? 1 : 0,
                       tele.vcuHvVoltage,
                       feedbackAgeMs,
                       static_cast<unsigned long>(leafFbDiag.rpm_frames),
                       id1daAgeMs,
                       static_cast<unsigned long>(leafFbDiag.torque_frames),
                       id1dbAgeMs,
                       static_cast<unsigned long>(leafFbDiag.temps_frames),
                       id1dcAgeMs,
                       static_cast<unsigned long>(leafFbDiag.status_frames),
                       id1d4AgeMs,
                       static_cast<unsigned long>(canStatsDiag.rxFrames),
                       static_cast<unsigned long>(canStatsDiag.rxLeafFrames),
                       static_cast<unsigned long>(canStatsDiag.rxUnknownFrames),
                       static_cast<unsigned long>(canStatsDiag.txFrames),
                       static_cast<unsigned long>(canStatsDiag.lastRxId));
        s_leafPreStatusLastMs = now;
    }
#endif

    if (s_leafRxDiagLastMs == 0U || (now - s_leafRxDiagLastMs) >= CAN_RX_CHECK_PERIOD_MS) {
        const bool id1daFresh = (leafFbDiag.rpm_update_ms != 0U) &&
                                (elapsedMsSafe(now, leafFbDiag.rpm_update_ms) <= CAN_RX_TARGET_MAX_AGE_MS);

        if (!id1daFresh) {
            if (!s_leafCanPartnerSeen) {
                if (s_leafRxAwaitPartnerLastMs == 0U ||
                    (now - s_leafRxAwaitPartnerLastMs) >= kLeafRxAwaitPartnerLogMs) {
                    Serial.println("[VCM-RX] Waiting for Leaf partner frame 1DA (RPM) not seen yet");
                    Serial0.println("[VCM-RX] Waiting for Leaf partner frame 1DA (RPM) not seen yet");
                    s_leafRxAwaitPartnerLastMs = now;
                }
            } else if (s_leafRxWarnLastMs == 0U || (now - s_leafRxWarnLastMs) >= CAN_RX_MISSING_LOG_PERIOD_MS) {
                Serial.printf("[VCM-RX] Missing/old required frame: 1DA=%d age_ms(1DA=%lu)\n",
                              id1daFresh ? 0 : 1,
                              static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.rpm_update_ms)));
                Serial0.printf("[VCM-RX] Missing/old required frame: 1DA=%d age_ms(1DA=%lu)\n",
                               id1daFresh ? 0 : 1,
                               static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.rpm_update_ms)));
                s_leafRxWarnLastMs = now;
            }
        } else {
            s_leafRxWarnLastMs = 0U;
        }

        s_leafRxDiagLastMs = now;
    }

#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
    {
        const MetaSense::CANBus::Stats& canStatsAlt = MetaSense::CANBus::stats();
        const bool altFramesChanged = (canStatsAlt.rx120Frames != lastCanAlt120FramesLogged) ||
                                      (canStatsAlt.rx55aFrames != lastCanAlt55aFramesLogged);
        const bool altDiagDue = (lastCanAltDiagMs == 0U) ||
                                ((now - lastCanAltDiagMs) >= CAN_ALT_DIAG_LOG_PERIOD_MS);
        if (altFramesChanged && altDiagDue) {
            const uint32_t dtMs = (lastCanAltDiagMs == 0U) ? 0U : (now - lastCanAltDiagMs);
            const uint32_t d120 = canStatsAlt.rx120Frames - lastCanAlt120FramesLogged;
            const uint32_t d55a = canStatsAlt.rx55aFrames - lastCanAlt55aFramesLogged;
            const float hz120 = (dtMs > 0U) ? (1000.0f * static_cast<float>(d120) / static_cast<float>(dtMs)) : 0.0f;
            const float hz55a = (dtMs > 0U) ? (1000.0f * static_cast<float>(d55a) / static_cast<float>(dtMs)) : 0.0f;

            char payload120[3 * 8] = {0};
            char payload55a[3 * 8] = {0};
            formatCanPayloadHex(canStatsAlt.last120Data,
                                canStatsAlt.last120Len,
                                payload120,
                                sizeof(payload120));
            formatCanPayloadHex(canStatsAlt.last55aData,
                                canStatsAlt.last55aLen,
                                payload55a,
                                sizeof(payload55a));

            Serial.printf("[VCM-ALT] id120=%lu(d=%lu,%.1fHz,age=%lu,len=%u,data=%s) id55A=%lu(d=%lu,%.1fHz,age=%lu,len=%u,data=%s)\n",
                          static_cast<unsigned long>(canStatsAlt.rx120Frames),
                          static_cast<unsigned long>(d120),
                          hz120,
                          static_cast<unsigned long>(elapsedMsSafe(now, canStatsAlt.last120Ms)),
                          static_cast<unsigned>(canStatsAlt.last120Len),
                          payload120,
                          static_cast<unsigned long>(canStatsAlt.rx55aFrames),
                          static_cast<unsigned long>(d55a),
                          hz55a,
                          static_cast<unsigned long>(elapsedMsSafe(now, canStatsAlt.last55aMs)),
                          static_cast<unsigned>(canStatsAlt.last55aLen),
                          payload55a);
            Serial0.printf("[VCM-ALT] id120=%lu(d=%lu,%.1fHz,age=%lu,len=%u,data=%s) id55A=%lu(d=%lu,%.1fHz,age=%lu,len=%u,data=%s)\n",
                           static_cast<unsigned long>(canStatsAlt.rx120Frames),
                           static_cast<unsigned long>(d120),
                           hz120,
                           static_cast<unsigned long>(elapsedMsSafe(now, canStatsAlt.last120Ms)),
                           static_cast<unsigned>(canStatsAlt.last120Len),
                           payload120,
                           static_cast<unsigned long>(canStatsAlt.rx55aFrames),
                           static_cast<unsigned long>(d55a),
                           hz55a,
                           static_cast<unsigned long>(elapsedMsSafe(now, canStatsAlt.last55aMs)),
                           static_cast<unsigned>(canStatsAlt.last55aLen),
                           payload55a);

            lastCanAlt120FramesLogged = canStatsAlt.rx120Frames;
            lastCanAlt55aFramesLogged = canStatsAlt.rx55aFrames;
            lastCanAltDiagMs = now;
        }
    }
#endif

    const bool leafTxChecklistActive = kLeafCanTxActive ||
                                       (kLeafCanHandshakeOnFirst1da && s_leafHandshakeSent);

    if (!leafTxChecklistActive) {
        if (!s_leafListenOnlyLogPrinted) {
            Serial.println("[VCM] Passive RX mode active: periodic TX checklist/state machine disabled");
            Serial0.println("[VCM] Passive RX mode active: periodic TX checklist/state machine disabled");
            s_leafListenOnlyLogPrinted = true;
        }

        if (kLeafCanHandshakeOnFirst1da && s_leafHandshakeArmed) {
            const bool decodedSecondaryFramesSeen = (lastCanTorqueFrameMs != 0U) ||
                                                    (lastCanTempsFrameMs != 0U) ||
                                                    (lastCanStatusFrameMs != 0U);
            if (decodedSecondaryFramesSeen) {
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
                Serial.printf("[VCM-HS] Handshake succeeded: decoded extra frame families (sent=%u attempts=%u)\n",
                              static_cast<unsigned>(s_leafHandshakeSentCount),
                              static_cast<unsigned>(s_leafHandshakeAttemptCount));
                Serial0.printf("[VCM-HS] Handshake succeeded: decoded extra frame families (sent=%u attempts=%u)\n",
                               static_cast<unsigned>(s_leafHandshakeSentCount),
                               static_cast<unsigned>(s_leafHandshakeAttemptCount));
#endif
                s_leafHandshakeArmed = false;
                s_leafHandshakeSent = true;
            }

            const bool attemptDue = (s_leafHandshakeLastAttemptMs == 0U) ||
                                    ((now - s_leafHandshakeLastAttemptMs) >= kLeafHandshakeAttemptPeriodMs);
            const bool withinWindow = (s_leafHandshakeStartMs == 0U) ||
                                      ((now - s_leafHandshakeStartMs) <= kLeafHandshakeWindowMs);
            if (s_leafHandshakeArmed &&
                attemptDue &&
                withinWindow &&
                s_leafHandshakeAttemptCount < kLeafHandshakeMaxAttempts) {
                s_leafHandshakeLastAttemptMs = now;
                ++s_leafHandshakeAttemptCount;
                const bool sent = MetaSense::Input::sendLeafTorqueCommand55B(0.0f,
                                                                             false,
                                                                             false,
                                                                             false,
                                                                             true);
                if (sent) {
                    ++s_leafHandshakeSentCount;
                }
            }

            if (s_leafHandshakeArmed &&
                (s_leafHandshakeSentCount >= kLeafHandshakeTargetSends ||
                 s_leafHandshakeAttemptCount >= kLeafHandshakeMaxAttempts ||
                 !withinWindow)) {
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
                Serial.printf("[VCM-HS] Handshake campaign complete: sent=%u attempts=%u window_ms=%lu\n",
                              static_cast<unsigned>(s_leafHandshakeSentCount),
                              static_cast<unsigned>(s_leafHandshakeAttemptCount),
                              static_cast<unsigned long>(now - s_leafHandshakeStartMs));
                Serial0.printf("[VCM-HS] Handshake campaign complete: sent=%u attempts=%u window_ms=%lu\n",
                               static_cast<unsigned>(s_leafHandshakeSentCount),
                               static_cast<unsigned>(s_leafHandshakeAttemptCount),
                               static_cast<unsigned long>(now - s_leafHandshakeStartMs));
#endif
                s_leafHandshakeArmed = false;
                s_leafHandshakeSent = (s_leafHandshakeSentCount > 0U);
            }
        }
    } else if ((now - lastLeafTxMs) >= CAN_TX_PERIOD_MS) {
        if (kLeafCanHandshakeOnFirst1da && !kLeafCanTxActive && !s_leafHandshakePromotedLogPrinted) {
            Serial.println("[VCM] Handshake complete: promoting to continuous 0x55B control mode");
            Serial0.println("[VCM] Handshake complete: promoting to continuous 0x55B control mode");
            s_leafHandshakePromotedLogPrinted = true;
        }

        const LeafVcmBringupState prevLeafVcmState = s_leafVcmState;
        if (lastLeafTxMs != 0U &&
            (now - lastLeafTxMs) > CAN_TX_MAX_GAP_MS &&
            s_leafCanPartnerSeen &&
            (s_leafVcmState != LeafVcmBringupState::CanOnline) &&
            !s_leafSimFeedbackActive) {
            s_leafTxGapFault = true;
            s_leafVcmFaultLatched = true;
            s_leafVcmLastFaultMs = now;
            s_leafVcmState = LeafVcmBringupState::Fault;
            s_leafVcmStateSinceMs = now;
        }

        lastLeafTxMs = now;

        const MetaSense::CANBus::Stats& canStats = MetaSense::CANBus::stats();
        const bool canBusReady = canStats.ready;
        if (canStats.busOffEvents != lastCanBusOffSeen ||
            canStats.statusQueryFailures != lastCanStatusQueryFailuresSeen) {
            s_leafVcmFaultLatched = true;
            s_leafVcmLastFaultMs = now;
            s_leafVcmState = LeafVcmBringupState::Fault;
            s_leafVcmStateSinceMs = now;
        }
        lastCanBusOffSeen = canStats.busOffEvents;
        lastCanStatusQueryFailuresSeen = canStats.statusQueryFailures;

        const LeafInvFeedback& aliveFb = MetaSense::CANBus::feedback();
        const bool nativeStatusAlive = (aliveFb.status_update_ms != 0U) &&
                                       (elapsedMsSafe(now, aliveFb.status_update_ms) <= kLeafVcmFeedbackTimeoutMs);
        const bool rpmAlive = (aliveFb.rpm_update_ms != 0U) &&
                              (elapsedMsSafe(now, aliveFb.rpm_update_ms) <= kLeafVcmFeedbackTimeoutMs);
        const bool fallbackAlive = rpmAlive;
        const bool leafFeedbackAlive = (lastCanLeafAnyUpdate != 0U) &&
                           (elapsedMsSafe(now, lastCanLeafAnyUpdate) <= kLeafVcmFeedbackTimeoutMs);
        const bool inverterAlive = nativeStatusAlive || (!nativeStatusAlive && fallbackAlive);
        if (leafFeedbackAlive || tele.leaf_invReady) {
            s_leafCanPartnerSeen = true;
        }
        const bool inverterFaulted = tele.leaf_invFault || tele.leaf_invLimp;
        const bool safetyOk = safe &&
                              tele.vcuReady &&
                              inverterAlive &&
                              canBusReady &&
                              !inverterFaulted;

        bool readyBit = false;
        bool hvOkBit = false;
        bool brakeBit = true;
        bool gearDriveBit = true;
        float torqueToSend = 0.0f;

#if METASENSE_LEAF_VCM_CHECKLIST_MODE
        switch (s_leafVcmState) {
            case LeafVcmBringupState::CanOnline:
                brakeBit = false;
                if (canBusReady &&
                    inverterAlive &&
                    ((now - s_leafVcmStateSinceMs) >= kLeafVcmCanOnlineDetectMs)) {
                    s_leafVcmState = LeafVcmBringupState::Precharge;
                    s_leafVcmStateSinceMs = now;
                }
                break;

            case LeafVcmBringupState::Precharge:
                brakeBit = false;
                if ((vcuDebugHvVoltage >= kLeafVcmHvReadyVoltageV) ||
                    ((now - s_leafVcmStateSinceMs) >= kLeafVcmPrechargeTimeoutMs)) {
                    s_leafVcmState = LeafVcmBringupState::HvOk;
                    s_leafVcmStateSinceMs = now;
                }
                break;

            case LeafVcmBringupState::HvOk:
                hvOkBit = true;
                brakeBit = false;
                if (safetyOk && ((now - s_leafVcmStateSinceMs) >= kLeafVcmHvOkSettleMs)) {
                    s_leafVcmState = LeafVcmBringupState::Ready;
                    s_leafVcmStateSinceMs = now;
                }
                break;

            case LeafVcmBringupState::Ready:
                readyBit = true;
                hvOkBit = true;
                brakeBit = (tele.mode == MetaSense::DynoMode::Brake);
                gearDriveBit = true;
                if (safetyOk) {
                    torqueToSend = torqueCmd;
                } else {
                    s_leafVcmState = LeafVcmBringupState::Fault;
                    s_leafVcmStateSinceMs = now;
                    s_leafVcmFaultLatched = true;
                    s_leafVcmLastFaultMs = now;
                    torqueToSend = 0.0f;
                    readyBit = false;
                    hvOkBit = false;
                    brakeBit = false;
                }
                break;

            case LeafVcmBringupState::Fault:
                readyBit = false;
                hvOkBit = false;
                brakeBit = false;
                gearDriveBit = true;
                torqueToSend = 0.0f;
                if (canBusReady &&
                    inverterAlive &&
                    !inverterFaulted &&
                    ((now - s_leafVcmStateSinceMs) >= kLeafVcmFaultRecoverMs)) {
                    s_leafTxGapFault = false;
                    s_leafVcmFaultLatched = false;
                    s_leafVcmState = LeafVcmBringupState::CanOnline;
                    s_leafVcmStateSinceMs = now;
                }
                break;
        }

        const bool allowInverterFaultLatch = s_leafCanPartnerSeen &&
                             !s_leafSimFeedbackActive &&
                                             (s_leafVcmState == LeafVcmBringupState::Precharge ||
                                              s_leafVcmState == LeafVcmBringupState::HvOk ||
                                              s_leafVcmState == LeafVcmBringupState::Ready ||
                                              s_leafVcmState == LeafVcmBringupState::Fault);
        if (allowInverterFaultLatch && inverterFaulted) {
            s_leafVcmState = LeafVcmBringupState::Fault;
            s_leafVcmStateSinceMs = now;
            s_leafVcmFaultLatched = true;
            s_leafVcmLastFaultMs = now;
            readyBit = false;
            hvOkBit = false;
            brakeBit = false;
            torqueToSend = 0.0f;
        }
#else
        const bool hvOk = tele.vcuReady;
        const bool inverterReadyBit = tele.leaf_invReady;
        readyBit = inverterReadyBit;
        hvOkBit = hvOk;
        brakeBit = (tele.mode == MetaSense::DynoMode::Brake);
        gearDriveBit = (tele.rpmTarget > 100.0f) || tele.recording;
        torqueToSend = (safe && hvOk && inverterReadyBit) ? torqueCmd : 0.0f;
#endif

        bool skipTxForGapTest = false;
#if METASENSE_LEAF_TX_GAP_TEST_ENABLE
        if (s_leafVcmState == LeafVcmBringupState::Ready) {
            if (s_leafTxGapTestCycleStartMs == 0U) {
                s_leafTxGapTestCycleStartMs = now;
                s_leafTxGapTestActive = false;
                s_leafTxGapTestLoggedStart = false;
                s_leafTxGapTestLoggedEnd = false;
            }

            const uint32_t elapsedCycleMs = now - s_leafTxGapTestCycleStartMs;
            if (elapsedCycleMs >= METASENSE_LEAF_TX_GAP_TEST_PERIOD_MS) {
                s_leafTxGapTestCycleStartMs = now;
                s_leafTxGapTestActive = true;
                s_leafTxGapTestLoggedStart = false;
                s_leafTxGapTestLoggedEnd = false;
            }

            if (s_leafTxGapTestActive) {
                const uint32_t gapElapsedMs = now - s_leafTxGapTestCycleStartMs;
                if (gapElapsedMs < METASENSE_LEAF_TX_GAP_TEST_DURATION_MS) {
                    skipTxForGapTest = true;
                    if (!s_leafTxGapTestLoggedStart) {
                        Serial.printf("[VCM-GAPTEST] Pausing 0x55B TX for %lu ms to provoke timeout bit\n",
                                      static_cast<unsigned long>(METASENSE_LEAF_TX_GAP_TEST_DURATION_MS));
                        Serial0.printf("[VCM-GAPTEST] Pausing 0x55B TX for %lu ms to provoke timeout bit\n",
                                       static_cast<unsigned long>(METASENSE_LEAF_TX_GAP_TEST_DURATION_MS));
                        s_leafTxGapTestLoggedStart = true;
                    }
                } else {
                    s_leafTxGapTestActive = false;
                    if (!s_leafTxGapTestLoggedEnd) {
                        Serial.println("[VCM-GAPTEST] 0x55B TX pause ended");
                        Serial0.println("[VCM-GAPTEST] 0x55B TX pause ended");
                        s_leafTxGapTestLoggedEnd = true;
                    }
                }
            }
        } else {
            s_leafTxGapTestCycleStartMs = now;
            s_leafTxGapTestActive = false;
            s_leafTxGapTestLoggedStart = false;
            s_leafTxGapTestLoggedEnd = false;
        }
#endif

        if (!skipTxForGapTest) {
            (void)sendLeafTorqueCommand55B(torqueToSend,
                                           readyBit,
                                           hvOkBit,
                                           brakeBit,
                                           gearDriveBit);
        }

        if (s_leafVcmState != prevLeafVcmState) {
            Serial.printf("[VCM-STATE] %s -> %s | can_ready=%d feedback_ms=%lu hv=%.1f vcu_ready=%d inv_ready=%d fault=%d\n",
                          leafVcmStateName(prevLeafVcmState),
                          leafVcmStateName(s_leafVcmState),
                          MetaSense::CANBus::isReady() ? 1 : 0,
                          static_cast<unsigned long>(elapsedMsSafe(now, lastCanLeafAnyUpdate)),
                          vcuDebugHvVoltage,
                          tele.vcuReady ? 1 : 0,
                          tele.leaf_invReady ? 1 : 0,
                          (tele.leaf_invFault || tele.leaf_invLimp || s_leafTxGapFault) ? 1 : 0);
            Serial0.printf("[VCM-STATE] %s -> %s | can_ready=%d feedback_ms=%lu hv=%.1f vcu_ready=%d inv_ready=%d fault=%d\n",
                           leafVcmStateName(prevLeafVcmState),
                           leafVcmStateName(s_leafVcmState),
                           MetaSense::CANBus::isReady() ? 1 : 0,
                           static_cast<unsigned long>(elapsedMsSafe(now, lastCanLeafAnyUpdate)),
                           vcuDebugHvVoltage,
                           tele.vcuReady ? 1 : 0,
                           tele.leaf_invReady ? 1 : 0,
                           (tele.leaf_invFault || tele.leaf_invLimp || s_leafTxGapFault) ? 1 : 0);
        }
    }

    {
        const MetaSense::CANBus::Stats& canStats = MetaSense::CANBus::stats();
        const bool canDiagChanged = !lastCanDiagInitialized ||
                                    (canStats.ready != lastCanDiagReady) ||
                                    (canStats.lastTwaiState != lastCanDiagState) ||
                                    (canStats.txFailures != lastCanDiagTxFailures) ||
                                    (canStats.txWhileNotReady != lastCanDiagTxWhileNotReady) ||
                                    (canStats.recoveries != lastCanDiagRecoveries) ||
                                    (canStats.busOffEvents != lastCanDiagBusOff) ||
                                    (canStats.statusQueryFailures != lastCanDiagStatusQueryFailures) ||
                                    (canStats.twaiRxOverrun != lastCanDiagTwaiRxOverrun) ||
                                    (canStats.twaiArbLost != lastCanDiagTwaiArbLost) ||
                                    (canStats.twaiBusError != lastCanDiagTwaiBusError) ||
                                    (canStats.twaiTxErrorCounter != lastCanDiagTwaiTxErr) ||
                                    (canStats.twaiRxErrorCounter != lastCanDiagTwaiRxErr);

        const bool canEventLogDue = (lastCanEventLogMs == 0U) ||
                                    ((now - lastCanEventLogMs) >= CAN_EVENT_LOG_MIN_PERIOD_MS);

        if (canDiagChanged && canEventLogDue) {
            Serial.printf("[CAN-EVENT] ready=%d state=%u tx_fail=%lu tx_not_ready=%lu recov=%lu bus_off=%lu status_q_fail=%lu twai(rxq=%lu txq=%lu rx_miss=%lu rx_ovr=%lu arb_lost=%lu bus_err=%lu tec=%lu rec=%lu)\n",
                          canStats.ready ? 1 : 0,
                          static_cast<unsigned>(canStats.lastTwaiState),
                          static_cast<unsigned long>(canStats.txFailures),
                          static_cast<unsigned long>(canStats.txWhileNotReady),
                          static_cast<unsigned long>(canStats.recoveries),
                          static_cast<unsigned long>(canStats.busOffEvents),
                          static_cast<unsigned long>(canStats.statusQueryFailures),
                          static_cast<unsigned long>(canStats.twaiRxQueued),
                          static_cast<unsigned long>(canStats.twaiTxQueued),
                          static_cast<unsigned long>(canStats.twaiRxMissed),
                          static_cast<unsigned long>(canStats.twaiRxOverrun),
                          static_cast<unsigned long>(canStats.twaiArbLost),
                          static_cast<unsigned long>(canStats.twaiBusError),
                          static_cast<unsigned long>(canStats.twaiTxErrorCounter),
                          static_cast<unsigned long>(canStats.twaiRxErrorCounter));
            Serial0.printf("[CAN-EVENT] ready=%d state=%u tx_fail=%lu tx_not_ready=%lu recov=%lu bus_off=%lu status_q_fail=%lu twai(rxq=%lu txq=%lu rx_miss=%lu rx_ovr=%lu arb_lost=%lu bus_err=%lu tec=%lu rec=%lu)\n",
                           canStats.ready ? 1 : 0,
                           static_cast<unsigned>(canStats.lastTwaiState),
                           static_cast<unsigned long>(canStats.txFailures),
                           static_cast<unsigned long>(canStats.txWhileNotReady),
                           static_cast<unsigned long>(canStats.recoveries),
                           static_cast<unsigned long>(canStats.busOffEvents),
                           static_cast<unsigned long>(canStats.statusQueryFailures),
                           static_cast<unsigned long>(canStats.twaiRxQueued),
                           static_cast<unsigned long>(canStats.twaiTxQueued),
                           static_cast<unsigned long>(canStats.twaiRxMissed),
                           static_cast<unsigned long>(canStats.twaiRxOverrun),
                           static_cast<unsigned long>(canStats.twaiArbLost),
                           static_cast<unsigned long>(canStats.twaiBusError),
                           static_cast<unsigned long>(canStats.twaiTxErrorCounter),
                           static_cast<unsigned long>(canStats.twaiRxErrorCounter));
            lastCanEventLogMs = now;
        }

        lastCanDiagInitialized = true;
        lastCanDiagReady = canStats.ready;
        lastCanDiagState = canStats.lastTwaiState;
        lastCanDiagTxFailures = canStats.txFailures;
        lastCanDiagTxWhileNotReady = canStats.txWhileNotReady;
        lastCanDiagRecoveries = canStats.recoveries;
        lastCanDiagBusOff = canStats.busOffEvents;
        lastCanDiagStatusQueryFailures = canStats.statusQueryFailures;
        lastCanDiagTwaiRxQueued = canStats.twaiRxQueued;
        lastCanDiagTwaiTxQueued = canStats.twaiTxQueued;
        lastCanDiagTwaiRxMissed = canStats.twaiRxMissed;
        lastCanDiagTwaiRxOverrun = canStats.twaiRxOverrun;
        lastCanDiagTwaiArbLost = canStats.twaiArbLost;
        lastCanDiagTwaiBusError = canStats.twaiBusError;
        lastCanDiagTwaiTxErr = canStats.twaiTxErrorCounter;
        lastCanDiagTwaiRxErr = canStats.twaiRxErrorCounter;
    }

    if (tele.recording) {
        appendFallbackRunLogSample(now);
        const float torqueMag = fabsf(tele.torqueNm);
        const float powerMag = fabsf(tele.kw);

        if (torqueMag > tele.peakTorque) {
            tele.peakTorque     = torqueMag;
            tele.peakTorque_RPM = tele.rpm;
        }
        if (powerMag > tele.peakKW) {
            tele.peakKW     = powerMag;
            tele.peakKW_RPM = tele.rpm;
        }
        if (tele.rpm > tele.maxRpm) {
            tele.maxRpm = tele.rpm;
        }
        if (torqueMag > tele.maxTorqueNm) {
            tele.maxTorqueNm = torqueMag;
        }
    }

    MetaSense::RunStorage::save(tele);

    if (prevRecording && !tele.recording) {
        commitFallbackRunLog("firmware_recording_stop");
        notifyRunComplete(tele);
    }
    prevRecording = tele.recording;
}

void publish()
{
    publishTelemetry();
}

} // namespace MetaSense::Input
