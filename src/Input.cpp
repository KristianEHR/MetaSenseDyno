#include "Input.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <freertos/semphr.h>
#include <math.h>
#include <esp_timer.h>
#include <Wire.h>
#include <cstdarg>

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
#include "LeafCrc.h"
#include "Leaf1d4ReplaySeries.h"
#include "CanConfig.h"
#include "FeatureFlags.h"
#include "TorqueConfig.h"
#include "LeafCanConfig.h"  // Phase 14C: Consolidated Leaf CAN protocol parameters
#include "globals.h"
#include "TelnetSerialBridge.h"

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
static TaskHandle_t leafTxPacerTaskHandle = nullptr;
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
// Production: diagnostics removed (METASENSE_STREAM_DIAGNOSTICS=0, METASENSE_JSON_DELAY_COUNTERS=0)
static float ambientTempC = 20.0f;
static float ambientHumidityPct = 50.0f;
static float ambientPressureHpa = 1013.25f;
static uint32_t lastAmbientSampleMs = 0;

// CAN RPM validity timeout + plausibility
static uint32_t lastCanRpmUpdate   = 0;
static const uint32_t CAN_RPM_TIMEOUT_MS = 100;
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
static float s_torqueStepSeqNm = 0.0f;
static int8_t s_torqueStepSeqDir = 1;
static uint32_t s_torqueStepSeqLastMs = 0;
static float s_leafLastSentTorqueNm = 0.0f;
static uint32_t s_leafLastSentTorqueMs = 0;
static uint8_t s_leaf1d4RollingCounter = 0U;
static uint8_t s_leaf1d4ReplayIndex = 0U;
static uint8_t s_leafLast1d4TxData[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
static uint8_t s_leafLast1d4TxLen = 0U;
static uint32_t s_leafLast1d4TxMs = 0;
// 0x1D4 Payload State Machine (independent of 10ms TX cadence)
static uint32_t s_leaf1d4PayloadStateLastUpdateMs = 0;  // Track last payload state machine update (100ms)
static uint8_t s_leaf1d4PayloadCachedFrameData[8] = {0U};  // Cached frame ready to transmit every 10ms
static float s_leaf1d4PayloadTorqueNm = 0.0f;  // Current cached torque
static int16_t s_leaf1d4PayloadTorqueRaw = 0;
static bool s_leaf1d4PayloadHvStatus = false;
static bool s_leaf1d4PayloadRelayPlus = false;
static uint8_t s_leaf1d4PayloadChargeStatus = 0U;
static uint8_t s_leaf1d4PayloadClock = 0U;
static uint8_t s_leaf1d4PayloadCrc = 0U;
static uint8_t s_leaf1d4PayloadCrcCalc = 0U;
static int8_t s_leaf1d4PayloadCrcOk = -1;
static uint32_t s_leaf1d4PayloadMs = 0;
// Torque source selection: true = automatic (PI control), false = manual (user command)
static bool s_leaf1d4TorqueSourceAutomatic = true;
static bool s_leaf1d4TxUsedRingBase = false;
static uint8_t s_leaf1d4TxRingBaseLen = 0U;
static uint8_t s_leaf1d4TxRingSourceAge = 0xFFU;
static uint32_t s_leaf1d4TxRingFallbackCount = 0U;
static uint32_t s_leafTorqueTrackStartMs = 0;
static uint16_t s_leafTorqueTrackLatencyMs = 0;
static bool s_leafTorqueTrackPending = false;
static float s_leafTorqueTrackTargetNm = 0.0f;
static bool vcuDebugRPlus = false;
static bool vcuDebugPrecharge = false;
static bool vcuDebugSsr = false;
static bool vcuDebugRMinus = false;
static uint32_t lastLeafTxMs = 0;
static uint32_t lastLeaf11aTxMs = 0;
static uint8_t s_leaf11aMuxTxSeq = 0U;
static uint8_t s_leaf11aMuxSeenMask = 0U;
static uint8_t s_leaf11aMuxRxSeenMask = 0U;
static bool s_leaf11aMuxTemplatesLocked = false;
static uint32_t s_leaf11aMuxTemplateLastRxFrames = 0U;
static uint8_t s_leaf11aMuxTemplateBySel[4][8] = {{0U}};
static uint32_t lastLeaf1d4MonitorSampleMs = 0;
static bool lastCanDiagInitialized = false;
static bool lastCanDiagReady = false;
static uint8_t lastCanDiagState = 0xFF;
static uint32_t lastCanDiagTxFrames = 0;
static uint32_t lastCanDiagTx1d4Frames = 0;
static uint32_t lastCanDiagTx11aFrames = 0;
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
static uint8_t lastCanDiag1d4TxData[8] = {0U};
static uint8_t lastCanDiag11aTxData[8] = {0U};
static uint8_t lastCanDiag1d4TxLen = 0U;
static uint8_t lastCanDiag11aTxLen = 0U;
static uint32_t s_leaf1d4FreshLastLogMs = 0U;
static uint32_t s_leaf1d4FreshLastTxCount = 0U;
static uint8_t s_leaf1d4FreshPrevData[8] = {0U};
static uint8_t s_leaf1d4FreshPrevLen = 0U;
static uint32_t s_leaf1d4FreshStaticRun = 0U;
// Production: CRC proof-testing removed (METASENSE_LEAF_VCM_DIAGNOSTICS=0)
// Production: all CRC diagnostics removed (METASENSE_LEAF_VCM_DIAGNOSTICS=0)
static uint32_t lastCanBusOffSeen = 0;
static uint32_t lastCanStatusQueryFailuresSeen = 0;

// RPM delta error
static bool  rpmDeltaError         = false;
static bool  canFallbackActive     = false;
static bool  activeRpmFromCan      = false;
static const float RPM_DELTA_LIMIT = 100.0f;
static const uint32_t CAN_TEMP_TIMEOUT_MS = 1000;
static const uint32_t CAN_TX_PERIOD_MS = 10;
static const uint32_t LEAF_1D4_TORQUE_PAYLOAD_UPDATE_PERIOD_MS = 100;  // Update torque payload every 100ms, send frame every 10ms
static const uint32_t LEAF_1D4_MONITOR_SAMPLE_PERIOD_MS = 100;
static const uint32_t CAN_RX_CHECK_PERIOD_MS = 20;
static const uint32_t CAN_RX_TARGET_MAX_AGE_MS = 250;
static const uint32_t CAN_RX_MISSING_LOG_PERIOD_MS = 5000;
static const uint32_t CAN_EVENT_LOG_MIN_PERIOD_MS = 5000;
static const uint32_t CAN_1DA_CRC_BAD_STREAK_LIMIT = 10;

#ifndef METASENSE_LEAF_CRC_CANDIDATE_HUNT
// Disable reverse-engineering candidate sweeps in normal operation.
#define METASENSE_LEAF_CRC_CANDIDATE_HUNT 0
#endif

// DEAD: METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE (Phase 15 cleanup) - no usage
// kLeafTxSuppressedForSniff logic removed; sniffing now only controlled by listen_only mode

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
static uint32_t s_leafRxDiagLastMs = 0;
static uint32_t s_leafRxWarnLastMs = 0;
static uint32_t s_leafRxAwaitPartnerLastMs = 0;
static uint32_t s_leafTxGapTestCycleStartMs = 0;
static bool s_leafTxGapTestActive = false;
static bool s_leafTxGapTestLoggedStart = false;
static bool s_leafTxGapTestLoggedEnd = false;
static volatile float s_leafUiTorqueDemandNm = 0.0f;
static volatile bool s_leafManualTorqueMode = false;  // true=manual, false=auto (PI controller)
static const char* s_lastHardwareState = nullptr;     // Track state for INIT entry detection
static volatile bool s_leafTxPacerEnabled = false;
static volatile float s_leafTxPacerTorqueNm = 0.0f;
static uint8_t s_torqueNibbleCounter = 1U;
static int8_t s_torqueNibbleDir = 1;
static uint32_t s_torqueNibbleLastStepMs = 0U;
static uint32_t s_hcmClockLastStepMs = 0U;
static volatile bool s_leafTxPacerReadyBit = false;
static volatile bool s_leafTxPacerHvOkBit = false;
static volatile bool s_leafTxPacerBrakeBit = false;
static volatile bool s_leafTxPacerGearDriveBit = true;

constexpr uint32_t kLeafVcmCanOnlineDetectMs = 500;
constexpr uint32_t kLeafVcmFeedbackTimeoutMs = 250;
constexpr uint32_t kLeafVcmPrechargeTimeoutMs = 2500;
constexpr float kLeafVcmHvReadyVoltageV = 300.0f;
constexpr uint32_t kLeafVcmHvOkSettleMs = 100;
constexpr uint32_t kLeafVcmFaultRecoverMs = 500;
constexpr float kLeafTorqueTrackStepNm = 2.0f;
constexpr float kLeafTorqueTrackAbsTolNm = 3.0f;
constexpr float kLeafTorqueTrackRelTol = 0.10f;
constexpr uint32_t kLeafTorqueTrackTimeoutMs = METASENSE_LEAF_TORQUE_TRACK_TIMEOUT_MS;
constexpr uint32_t kLeafSimFeedbackPeriodMs = 20;
constexpr uint32_t kLeafSimBootDelayMs = 1000;
constexpr uint32_t kLeafSimReadyDelayMs = 1800;
constexpr uint32_t kLeafRxAwaitPartnerLogMs = 5000;
constexpr uint16_t kLeafHandshakeTargetSends = 80;
constexpr uint16_t kLeafHandshakeMaxAttempts = 200;
constexpr uint32_t kLeafHandshakeWindowMs = 5000;
constexpr uint32_t kLeafHandshakeAttemptPeriodMs = 20;

// Phase 14C: Leaf CAN protocol parameters now consolidated in include/LeafCanConfig.h
// Removed ~130 lines of duplicated #ifndef/#define METASENSE_LEAF_* blocks

constexpr bool kLeafCanHandshakeOnFirst1da = (METASENSE_LEAF_CAN_HANDSHAKE_ON_FIRST_1DA != 0);
constexpr bool kLeafCanTxActive = (METASENSE_LEAF_CAN_TX_ENABLED != 0) &&
                                  (METASENSE_LEAF_CAN_LISTEN_ONLY == 0) &&
                                  !kLeafCanHandshakeOnFirst1da;
constexpr bool kLeaf11aTxEnabled = (METASENSE_LEAF_11A_TX_ENABLED != 0) &&
                                   (METASENSE_LEAF_CAN_LISTEN_ONLY == 0) &&
                                   (METASENSE_LEAF_CAN_TX_ENABLED != 0);
constexpr uint32_t kLeaf11aTxPeriodMs = METASENSE_LEAF_11A_TX_PERIOD_MS;

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
#define METASENSE_WS_FAST_PERIOD_MS 20  // 50 Hz - ultra responsive updates
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

void setLeafUiTorqueDemandNmInternal(float torqueNm)
{
    const float torqueFinite = isfinite(torqueNm) ? torqueNm : 0.0f;
    s_leafUiTorqueDemandNm = constrain(torqueFinite, -512.0f, 511.75f);
}

float getLeafUiTorqueDemandNmInternal()
{
    return s_leafUiTorqueDemandNm;
}

uint32_t elapsedMsSafe(uint32_t now, uint32_t since)
{
    if (since == 0U || now < since) {
        return 0U;
    }
    return now - since;
}

bool is1daWireCrcTrustedForFallback(const MetaSense::CANBus::Stats& canStats, uint32_t now)
{
    static uint32_t s_lastEvaluatedCrcFrames = 0U;
    static uint32_t s_badCrcStreak = 0U;
    static bool s_crcTrustLatched = false;

    const bool fresh1da = (canStats.last1daMs != 0U) &&
                          (elapsedMsSafe(now, canStats.last1daMs) <= CAN_RX_TARGET_MAX_AGE_MS);
    if (!fresh1da) {
        s_crcTrustLatched = false;
        s_badCrcStreak = 0U;
        return false;
    }

    const uint32_t totalCrcFrames = canStats.rx1daWireCrcOkFrames + canStats.rx1daWireCrcBadFrames;
    if (totalCrcFrames < s_lastEvaluatedCrcFrames) {
        // Counter reset (e.g. reboot/reset); restart latching state.
        s_lastEvaluatedCrcFrames = totalCrcFrames;
        s_badCrcStreak = 0U;
        s_crcTrustLatched = false;
    }

    if (totalCrcFrames != s_lastEvaluatedCrcFrames) {
        const uint32_t deltaFrames = totalCrcFrames - s_lastEvaluatedCrcFrames;
        s_lastEvaluatedCrcFrames = totalCrcFrames;

        if (canStats.last1daWireCrcOk == 1) {
            s_badCrcStreak = 0U;
            s_crcTrustLatched = true;
        } else if (canStats.last1daWireCrcOk == 0) {
            const uint32_t cappedDelta = (deltaFrames > CAN_1DA_CRC_BAD_STREAK_LIMIT)
                ? CAN_1DA_CRC_BAD_STREAK_LIMIT
                : deltaFrames;
            s_badCrcStreak = (s_badCrcStreak + cappedDelta > CAN_1DA_CRC_BAD_STREAK_LIMIT)
                ? CAN_1DA_CRC_BAD_STREAK_LIMIT
                : (s_badCrcStreak + cappedDelta);
            if (s_badCrcStreak >= CAN_1DA_CRC_BAD_STREAK_LIMIT) {
                s_crcTrustLatched = false;
            }
        }
    }

    return s_crcTrustLatched;
}

struct Leaf120CommandDecode {
    uint16_t torqueDemandRawBe = 0U;
    int16_t torqueDemandSignedBe = 0;
    float torqueDemandNmBase = 0.0f;
    uint8_t unknown120_2 = 0U;
    uint8_t crc120 = 0U;
};


uint32_t extractIntelUnsigned(const uint8_t* data, uint8_t len, uint8_t startBit, uint8_t bitLen)
{
    if (data == nullptr || bitLen == 0U || bitLen > 32U) {
        return 0U;
    }
    const uint16_t endBit = static_cast<uint16_t>(startBit) + static_cast<uint16_t>(bitLen - 1U);
    if (len == 0U || (endBit / 8U) >= len) {
        return 0U;
    }

    uint32_t value = 0U;
    for (uint8_t i = 0U; i < bitLen; ++i) {
        const uint16_t bit = static_cast<uint16_t>(startBit) + i;
        const uint8_t byteIndex = static_cast<uint8_t>(bit / 8U);
        const uint8_t bitIndex = static_cast<uint8_t>(bit % 8U);
        const uint32_t bitVal = (static_cast<uint32_t>(data[byteIndex]) >> bitIndex) & 0x01U;
        value |= (bitVal << i);
    }
    return value;
}

void setIntelUnsigned(uint8_t* data, uint8_t len, uint8_t startBit, uint8_t bitLen, uint32_t value)
{
    if (data == nullptr || bitLen == 0U || bitLen > 32U) {
        return;
    }
    const uint16_t endBit = static_cast<uint16_t>(startBit) + static_cast<uint16_t>(bitLen - 1U);
    if (len == 0U || (endBit / 8U) >= len) {
        return;
    }

    for (uint8_t i = 0U; i < bitLen; ++i) {
        const uint16_t bit = static_cast<uint16_t>(startBit) + i;
        const uint8_t byteIndex = static_cast<uint8_t>(bit / 8U);
        const uint8_t bitIndex = static_cast<uint8_t>(bit % 8U);
        const uint8_t bitVal = static_cast<uint8_t>((value >> i) & 0x01U);
        if (bitVal != 0U) {
            data[byteIndex] = static_cast<uint8_t>(data[byteIndex] | static_cast<uint8_t>(1U << bitIndex));
        } else {
            data[byteIndex] = static_cast<uint8_t>(data[byteIndex] & static_cast<uint8_t>(~(1U << bitIndex)));
        }
    }
}

void setMotorolaUnsigned(uint8_t* data, uint8_t len, int startBit, uint8_t bitLen, uint32_t value)
{
    if (data == nullptr || bitLen == 0U || bitLen > 32U || startBit < 0) {
        return;
    }

    int bit = startBit;
    for (uint8_t i = 0U; i < bitLen; ++i) {
        if (bit < 0) {
            return;
        }
        const int byteIndex = bit / 8;
        const int bitIndex = bit % 8;
        if (byteIndex < 0 || byteIndex >= static_cast<int>(len)) {
            return;
        }

        const uint8_t srcBit = static_cast<uint8_t>(bitLen - 1U - i);
        const uint8_t bitVal = static_cast<uint8_t>((value >> srcBit) & 0x01U);
        if (bitVal != 0U) {
            data[byteIndex] = static_cast<uint8_t>(data[byteIndex] | static_cast<uint8_t>(1U << bitIndex));
        } else {
            data[byteIndex] = static_cast<uint8_t>(data[byteIndex] & static_cast<uint8_t>(~(1U << bitIndex)));
        }

        if ((bit % 8) == 0) {
            bit += 15;
        } else {
            --bit;
        }
    }
}

uint32_t extractMotorolaUnsigned(const uint8_t* data, uint8_t len, int startBit, uint8_t bitLen)
{
    if (data == nullptr || bitLen == 0U || bitLen > 32U || startBit < 0) {
        return 0U;
    }

    uint32_t value = 0U;
    int bit = startBit;
    for (uint8_t i = 0U; i < bitLen; ++i) {
        if (bit < 0) {
            return 0U;
        }
        const int byteIndex = bit / 8;
        const int bitIndex = bit % 8;
        if (byteIndex < 0 || byteIndex >= static_cast<int>(len)) {
            return 0U;
        }
        const uint32_t bitVal = (static_cast<uint32_t>(data[byteIndex]) >> bitIndex) & 0x01U;
        value = (value << 1) | bitVal;

        if ((bit % 8) == 0) {
            bit += 15;
        } else {
            --bit;
        }
    }

    return value;
}

int32_t signExtendBits(uint32_t raw, uint8_t bitLen)
{
    if (bitLen == 0U || bitLen >= 32U) {
        return static_cast<int32_t>(raw);
    }
    const uint32_t signBit = 1UL << (bitLen - 1U);
    if ((raw & signBit) == 0U) {
        return static_cast<int32_t>(raw);
    }
    const uint32_t mask = (1UL << bitLen) - 1UL;
    return static_cast<int32_t>(raw | ~mask);
}

uint8_t crc8MsbGeneric(const uint8_t* data, uint8_t len, uint8_t poly, uint8_t init, uint8_t xorOut)
{
    uint8_t crc = init;
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            const bool msb = (crc & 0x80U) != 0U;
            crc <<= 1;
            if (msb) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint8_t>(crc ^ xorOut);
}

uint8_t crc8LsbGeneric(const uint8_t* data, uint8_t len, uint8_t poly, uint8_t init, uint8_t xorOut)
{
    uint8_t crc = init;
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            const bool lsb = (crc & 0x01U) != 0U;
            crc >>= 1;
            if (lsb) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint8_t>(crc ^ xorOut);
}

uint8_t computeApprovedLeafFrameCrc(uint8_t idLo, const uint8_t* payload8)
{
    if (idLo == 0xDAU) {
        return MetaSense::LeafCRC::computeExact1daWireCrc(idLo, payload8);
    }
    if (idLo == 0xD4U) {
        // For 0x1D4, use the correct wire CRC with polynomial 0x1D29
        // The counter in byte 5 upper nibble is included in the calculation
        return MetaSense::LeafCRC::computeExact1d4LikeCrc(idLo, payload8);
    }
    return MetaSense::LeafCRC::computeExact1d4LikeCrc(idLo, payload8);
}

uint8_t computeLeaf1d4CrcConformant(const uint8_t* payload7)
{
    if (payload7 == nullptr) {
        return 0U;
    }
    return computeApprovedLeafFrameCrc(0xD4U, payload7);
}

uint8_t computeLeaf1daCrcForMonitor(const uint8_t* payload8, uint8_t payloadLen)
{
    if (payload8 == nullptr || payloadLen < 8U) {
        return 0U;
    }
    return computeApprovedLeafFrameCrc(0xDAU, payload8);
}

uint8_t computeLeaf11aCrcBaseIdLo(const uint8_t* payload7)
{
    return MetaSense::LeafCRC::computeBaseIdLo(0x1AU, payload7);
}

uint8_t computeLeaf11aCrcConformant(const uint8_t* payload7,
                                    const uint8_t* template8)
{
    if (payload7 == nullptr) {
        return 0U;
    }
    (void)template8;
    return computeApprovedLeafFrameCrc(0x1AU, payload7);
}

struct Leaf1daMonitorDecode {
    float inputVoltage = 0.0f;
    float rpm = 0.0f;
    float torqueNm = 0.0f;
    uint8_t clock = 0U;
    uint8_t errorCodes = 0U;
    uint8_t invStatusBit = 0U;
    uint8_t invFaultMap = 0U;
    uint8_t invBlinky = 0U;
    uint16_t invUnknownFaults = 0U;
    uint8_t invFaultCanTimeoutMaybe = 0U;
    bool hasFrame = false;
    bool hasTorque = false;
};

Leaf1daMonitorDecode decodeLeaf1daMonitorFromRaw(const uint8_t* data, uint8_t len)
{
    Leaf1daMonitorDecode out;
    if (data == nullptr || len < 4U) {
        return out;
    }

    out.hasFrame = true;
    out.inputVoltage = static_cast<float>(data[0]) * 2.0f;

    const uint16_t raw01 = static_cast<uint16_t>(data[0]) |
                           (static_cast<uint16_t>(data[1]) << 8);
    const uint16_t raw23 = static_cast<uint16_t>(data[2]) |
                           (static_cast<uint16_t>(data[3]) << 8);
    const float legacyRpm = static_cast<float>((raw23 != 0U || raw01 == 0U) ? raw23 : raw01);
    const float ze1Rpm = static_cast<float>(signExtendBits(extractMotorolaUnsigned(data, len, 39, 15U), 15U));
    const bool ze1Valid = isfinite(ze1Rpm) && (fabsf(ze1Rpm) <= 20000.0f);
    const bool legacyValid = isfinite(legacyRpm) && (fabsf(legacyRpm) <= 20000.0f);
    out.rpm = ze1Valid ? ze1Rpm : (legacyValid ? legacyRpm : 0.0f);

    if (len < 8U) {
        return out;
    }

    out.clock = static_cast<uint8_t>(data[6] & 0x03U);
    const uint8_t stateNoClock = static_cast<uint8_t>(data[6] & 0xFCU);
    out.errorCodes = (stateNoClock == 0x24U || stateNoClock == 0x18U)
        ? 0U
        : static_cast<uint8_t>((data[6] >> 2) & 0x3FU);

    const float ze1Torque = static_cast<float>(signExtendBits(extractMotorolaUnsigned(data, len, 18, 11U), 11U)) * 0.5f;
    const int16_t raw01s = static_cast<int16_t>(raw01);
    const int16_t raw23s = static_cast<int16_t>(raw23);
    const float legacyTorque = static_cast<float>((raw23s != 0 || raw01s == 0) ? raw23s : raw01s) * 0.5f;
    out.torqueNm = (fabsf(ze1Torque) <= 500.0f) ? ze1Torque : legacyTorque;
    out.hasTorque = true;

    out.invStatusBit = static_cast<uint8_t>((data[5] >> 0U) & 0x01U);
    out.invFaultMap = static_cast<uint8_t>((data[6] >> 2U) & 0x3FU);
    out.invBlinky = static_cast<uint8_t>((data[1] >> 6U) & 0x03U);
    out.invUnknownFaults = static_cast<uint16_t>(extractMotorolaUnsigned(data, len, 13, 11U));
    out.invFaultCanTimeoutMaybe = static_cast<uint8_t>((data[2] >> 7U) & 0x01U);
    return out;
}

struct Leaf1d4FrameFields {
    float torqueDemandNm = 0.0f;
    int16_t torqueDemandRaw = 0;
    bool hvStatus = false;
    bool rbPlus = false;
    uint8_t chargeStatus = 0U;
    uint8_t cmdClock = 0U;
};

struct Leaf1d4CommandDecode {
    int16_t motorAmpTorqueRaw = 0;
    float motorAmpTorqueNm = 0.0f;
    uint8_t hcmClock = 0U;
    bool hvSupplyStatus = false;
    bool relayPlusStatus = false;
    uint8_t chargeStatus = 0U;
    uint8_t crc1d4 = 0U;
    bool valid = false;
};

Leaf1d4CommandDecode decodeLeaf1d4Command(const uint8_t* data, uint8_t len)
{
    Leaf1d4CommandDecode decoded;
    if (data == nullptr || len < 8U) {
        return decoded;
    }

    decoded.motorAmpTorqueRaw = static_cast<int16_t>(((static_cast<uint16_t>(data[2]) << 4) |
                                                      (static_cast<uint16_t>(data[3]) & 0x0FU)) & 0x0FFFU);
    if ((decoded.motorAmpTorqueRaw & 0x0800) != 0) {
        decoded.motorAmpTorqueRaw = static_cast<int16_t>(decoded.motorAmpTorqueRaw | static_cast<int16_t>(0xF000));
    }
    decoded.motorAmpTorqueNm = static_cast<float>(decoded.motorAmpTorqueRaw) * METASENSE_LEAF_1D4_TORQUE_LSB_NM;
    decoded.hcmClock = static_cast<uint8_t>(extractIntelUnsigned(data, len, 38U, 2U) & 0x03U);
    decoded.hvSupplyStatus = extractIntelUnsigned(data, len, 34U, 1U) != 0U;
    decoded.relayPlusStatus = extractIntelUnsigned(data, len, 46U, 1U) != 0U;
    decoded.chargeStatus = data[6];
    decoded.crc1d4 = data[7];
    decoded.valid = true;
    return decoded;
}

int16_t encodeLeaf1d4TorqueRaw(float torqueDemandNm)
{
    constexpr float kTorqueRawMin = -2048.0f * METASENSE_LEAF_1D4_TORQUE_LSB_NM;
    constexpr float kTorqueRawMax = 2047.0f * METASENSE_LEAF_1D4_TORQUE_LSB_NM;
    const float tqClamped = constrain(torqueDemandNm, kTorqueRawMin, kTorqueRawMax);
    return static_cast<int16_t>(lroundf(tqClamped / METASENSE_LEAF_1D4_TORQUE_LSB_NM));
}

void patchLeaf1d4TorqueFieldMotorola23_12(uint8_t (&frame)[8], int16_t torqueRaw)
{
    // DBC: SG_ MotorAmpTorqueRequest : 23|12@0- (0.25,0)
    // 12-bit signed two's complement placed as Motorola at start bit 23.
    const uint16_t raw12 = static_cast<uint16_t>(torqueRaw) & 0x0FFFU;
    frame[2] = static_cast<uint8_t>((raw12 >> 4) & 0xFFU);
    frame[3] = static_cast<uint8_t>((frame[3] & 0xF0U) | (raw12 & 0x000FU));
}

void patchLeaf1d4FrameFields(const Leaf1d4FrameFields& fields,
                             uint8_t (&out)[8])
{
    memset(out, 0, 8U);

    patchLeaf1d4TorqueFieldMotorola23_12(out, fields.torqueDemandRaw);
    setIntelUnsigned(out, 8U, 34U, 1U, fields.hvStatus ? 1U : 0U);
    setIntelUnsigned(out, 8U, 46U, 1U, fields.rbPlus ? 1U : 0U);
    out[6] = fields.chargeStatus;
    setIntelUnsigned(out, 8U, 38U, 2U, static_cast<uint32_t>(fields.cmdClock & 0x03U));
    out[7] = computeLeaf1d4CrcConformant(out);
}

uint8_t crc8Lsb(const uint8_t* data, uint8_t len, uint8_t poly, uint8_t init, uint8_t xorOut)
{
    uint8_t crc = init;
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            const bool lsb = (crc & 0x01U) != 0U;
            crc >>= 1;
            if (lsb) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint8_t>(crc ^ xorOut);
}

uint8_t crc8Msb(const uint8_t* data, uint8_t len, uint8_t poly, uint8_t init, uint8_t xorOut)
{
    uint8_t crc = init;
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            const bool msb = (crc & 0x80U) != 0U;
            crc <<= 1;
            if (msb) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint8_t>(crc ^ xorOut);
}

uint32_t computeLeafCrcCandidateMaskCommon(const uint8_t* payload,
                                           uint8_t payloadLen,
                                           uint8_t crcRx,
                                           uint16_t id)
{
    if (payload == nullptr || payloadLen == 0U) {
        return 0U;
    }

    uint8_t crcXor = 0U;
    uint8_t crcSum = 0U;
    for (uint8_t i = 0U; i < payloadLen; ++i) {
        crcXor ^= payload[i];
        crcSum = static_cast<uint8_t>(crcSum + payload[i]);
    }
    const uint8_t crcInvSum = static_cast<uint8_t>(~crcSum);
    const uint8_t crc07Init00 = crc8Msb(payload, payloadLen, 0x07U, 0x00U, 0x00U);
    const uint8_t crc07InitFF = crc8Msb(payload, payloadLen, 0x07U, 0xFFU, 0x00U);
    const uint8_t crc1DInitFF = crc8Msb(payload, payloadLen, 0x1DU, 0xFFU, 0xFFU);
    const uint8_t crc2FInitFF = crc8Msb(payload, payloadLen, 0x2FU, 0xFFU, 0xFFU);
    const uint8_t crc1DInit00 = crc8Msb(payload, payloadLen, 0x1DU, 0x00U, 0x00U);
    const uint8_t crc1DInitFFXor00 = crc8Msb(payload, payloadLen, 0x1DU, 0xFFU, 0x00U);

    uint8_t payloadWithIdBe[10] = {0U};
    uint8_t payloadWithIdLo[9] = {0U};
    const uint8_t idHi = static_cast<uint8_t>((id >> 8) & 0xFFU);
    const uint8_t idLo = static_cast<uint8_t>(id & 0xFFU);
    payloadWithIdBe[0] = idHi;
    payloadWithIdBe[1] = idLo;
    payloadWithIdLo[0] = idLo;
    memcpy(&payloadWithIdBe[2], payload, payloadLen);
    memcpy(&payloadWithIdLo[1], payload, payloadLen);

    const uint8_t crc1DIdBe = crc8Msb(payloadWithIdBe, static_cast<uint8_t>(payloadLen + 2U), 0x1DU, 0xFFU, 0xFFU);
    const uint8_t crc1DIdLo = crc8Msb(payloadWithIdLo, static_cast<uint8_t>(payloadLen + 1U), 0x1DU, 0xFFU, 0xFFU);
    const uint8_t crc1DRefInitFF = crc8Lsb(payload, payloadLen, 0xB8U, 0xFFU, 0xFFU);
    const uint8_t crc1DRefInitFFXor00 = crc8Lsb(payload, payloadLen, 0xB8U, 0xFFU, 0x00U);
    const uint8_t crc1DRefInit00 = crc8Lsb(payload, payloadLen, 0xB8U, 0x00U, 0x00U);
    const uint8_t crc1DRefIdBe = crc8Lsb(payloadWithIdBe, static_cast<uint8_t>(payloadLen + 2U), 0xB8U, 0xFFU, 0xFFU);
    const uint8_t crc1DRefIdLo = crc8Lsb(payloadWithIdLo, static_cast<uint8_t>(payloadLen + 1U), 0xB8U, 0xFFU, 0xFFU);
    const uint8_t crc2FRefInitFF = crc8Lsb(payload, payloadLen, 0xF4U, 0xFFU, 0xFFU);

    return static_cast<uint32_t>(
        ((crcRx == crcXor) ? (1U << 0) : 0U) |
        ((crcRx == crcSum) ? (1U << 1) : 0U) |
        ((crcRx == crcInvSum) ? (1U << 2) : 0U) |
        ((crcRx == crc07Init00) ? (1U << 3) : 0U) |
        ((crcRx == crc07InitFF) ? (1U << 4) : 0U) |
        ((crcRx == crc1DInitFF) ? (1U << 5) : 0U) |
        ((crcRx == crc2FInitFF) ? (1U << 6) : 0U) |
        ((crcRx == crc1DInit00) ? (1U << 7) : 0U) |
        ((crcRx == crc1DInitFFXor00) ? (1U << 8) : 0U) |
        ((crcRx == crc1DIdBe) ? (1U << 9) : 0U) |
        ((crcRx == crc1DIdLo) ? (1U << 10) : 0U) |
        ((crcRx == crc1DRefInitFF) ? (1U << 11) : 0U) |
        ((crcRx == crc1DRefInitFFXor00) ? (1U << 12) : 0U) |
        ((crcRx == crc1DRefInit00) ? (1U << 13) : 0U) |
        ((crcRx == crc1DRefIdBe) ? (1U << 14) : 0U) |
        ((crcRx == crc1DRefIdLo) ? (1U << 15) : 0U) |
        ((crcRx == crc2FRefInitFF) ? (1U << 16) : 0U));
}

uint32_t computeLeaf120CrcCandidateMask(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len < 4U) {
        return 0U;
    }

    return computeLeafCrcCandidateMaskCommon(data, 3U, data[3], 0x0120U);
}

uint32_t computeLeaf1daCrcCandidateMask(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len < 8U) {
        return 0U;
    }

    return computeLeafCrcCandidateMaskCommon(data, 7U, data[7], 0x01DAU);
}

uint32_t computeLeaf1d4CrcCandidateMask(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len < 8U) {
        return 0U;
    }

    return computeLeafCrcCandidateMaskCommon(data, 7U, data[7], 0x01D4U);
}

uint8_t countSetBitsU32(uint32_t v)
{
    uint8_t n = 0U;
    while (v != 0U) {
        n = static_cast<uint8_t>(n + static_cast<uint8_t>(v & 1U));
        v >>= 1;
    }
    return n;
}

int8_t firstSetBitU32(uint32_t v)
{
    for (uint8_t bit = 0U; bit < 32U; ++bit) {
        if ((v & (1UL << bit)) != 0U) {
            return static_cast<int8_t>(bit);
        }
    }
    return -1;
}

#if !(defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0))
constexpr uint8_t kLeaf1daResidueCandidateCount = 8U;
#endif

void computeLeaf1daResidues(const uint8_t* data, uint8_t len, uint8_t (&out)[kLeaf1daResidueCandidateCount])
{
    for (uint8_t i = 0U; i < kLeaf1daResidueCandidateCount; ++i) {
        out[i] = 0U;
    }

    if (data == nullptr || len < 8U) {
        return;
    }

    out[0] = crc8Msb(data, 8U, 0x07U, 0x00U, 0x00U);
    out[1] = crc8Msb(data, 8U, 0x07U, 0xFFU, 0x00U);
    out[2] = crc8Msb(data, 8U, 0x1DU, 0xFFU, 0xFFU);
    out[3] = crc8Msb(data, 8U, 0x1DU, 0xFFU, 0x00U);
    out[4] = crc8Msb(data, 8U, 0x2FU, 0xFFU, 0xFFU);
    out[5] = crc8Lsb(data, 8U, 0xB8U, 0xFFU, 0xFFU);
    out[6] = crc8Lsb(data, 8U, 0xB8U, 0xFFU, 0x00U);
    out[7] = crc8Lsb(data, 8U, 0xF4U, 0xFFU, 0xFFU);
}

uint32_t computeLeaf1daAutosarCandidateMask(const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len < 8U) {
        return 0U;
    }

    const uint8_t crcRx = data[7];
    const uint8_t mgClock = static_cast<uint8_t>(data[6] & 0x03U);
    const uint8_t d6NoClock = static_cast<uint8_t>(data[6] & 0xFCU);

    const uint8_t payloadIdBe[9] = {
        0x01U, 0xDAU,
        data[0], data[1], data[2], data[3], data[4], data[5], data[6]
    };
    const uint8_t payloadD6Masked[9] = {
        data[0], data[1], data[2], data[3], data[4], data[5], d6NoClock,
        0x01U, 0xDAU
    };
    const uint8_t payloadClockInjected[10] = {
        data[0], data[1], data[2], data[3], data[4], data[5], d6NoClock,
        mgClock, 0x01U, 0xDAU
    };

    const uint8_t c2FIdBe = crc8Msb(payloadIdBe, 9U, 0x2FU, 0xFFU, 0xFFU);
    const uint8_t c1DIdBe = crc8Msb(payloadIdBe, 9U, 0x1DU, 0xFFU, 0xFFU);
    const uint8_t c2FD6Masked = crc8Msb(payloadD6Masked, 9U, 0x2FU, 0xFFU, 0xFFU);
    const uint8_t c2FClockInjected = crc8Msb(payloadClockInjected, 10U, 0x2FU, 0xFFU, 0xFFU);

    return static_cast<uint32_t>(
        ((crcRx == c2FIdBe) ? (1U << 0) : 0U) |
        ((crcRx == c1DIdBe) ? (1U << 1) : 0U) |
        ((crcRx == c2FD6Masked) ? (1U << 2) : 0U) |
        ((crcRx == c2FClockInjected) ? (1U << 3) : 0U));
}

struct Leaf120TopValue {
    uint8_t value = 0U;
    uint32_t count = 0U;
};

uint8_t findTopByteValue(const uint32_t* counts, uint32_t& outCount)
{
    uint8_t topValue = 0U;
    outCount = 0U;
    if (counts == nullptr) {
        return topValue;
    }

    for (uint16_t i = 0U; i < 256U; ++i) {
        if (counts[i] > outCount) {
            outCount = counts[i];
            topValue = static_cast<uint8_t>(i);
        }
    }
    return topValue;
}

void findTopLeaf120Values(const uint32_t* counts, Leaf120TopValue (&top)[3])
{
    for (Leaf120TopValue& entry : top) {
        entry.value = 0U;
        entry.count = 0U;
    }

    for (uint16_t rawValue = 0U; rawValue < 256U; ++rawValue) {
        const uint32_t count = counts[rawValue];
        if (count == 0U) {
            continue;
        }

        for (size_t slot = 0U; slot < 3U; ++slot) {
            if (count > top[slot].count) {
                for (size_t shift = 2U; shift > slot; --shift) {
                    top[shift] = top[shift - 1U];
                }
                top[slot].value = static_cast<uint8_t>(rawValue);
                top[slot].count = count;
                break;
            }
        }
    }
}

void formatLeaf120TopValues(const uint32_t* counts, char* out, size_t outSize)
{
    if (outSize == 0U) {
        return;
    }

    out[0] = '\0';
    Leaf120TopValue top[3];
    findTopLeaf120Values(counts, top);
    if (top[0].count == 0U) {
        snprintf(out, outSize, "none");
        return;
    }

    size_t used = 0U;
    for (size_t i = 0U; i < 3U && top[i].count > 0U; ++i) {
        const int written = snprintf(out + used,
                                     outSize - used,
                                     (i == 0U) ? "0x%02X:%lu" : ",0x%02X:%lu",
                                     static_cast<unsigned>(top[i].value),
                                     static_cast<unsigned long>(top[i].count));
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
static bool prevStartRequestState = false;
static uint32_t startRequestDebounceMs = 0;
constexpr uint32_t kStartRequestDebounceThresholdMs = 40;
constexpr float kStartSwitchMotorRpm = 800.0f;
static bool s_startSwitchOverridePending = false;
static float s_startSwitchOverrideRpm = kStartSwitchMotorRpm;

static bool isStartControlWindow()
{
    const char* hwState = MetaSense::HardwareOutputStateMachine::stateName();
    if (hwState == nullptr) {
        return false;
    }
    return (strcmp(hwState, "INIT") == 0) ||
           (strcmp(hwState, "START") == 0) ||
           MetaSense::HardwareOutputStateMachine::hasPrestartWarning();
}

static uint32_t s_canStartRxFrames = 0U;
static uint32_t s_canStartLeafFrames = 0U;
static uint32_t s_canStart1daFrames = 0U;
static bool s_canStartReadyLatched = false;
static uint32_t s_initCanTxLastMs = 0U;

static bool evaluateCanStartReadiness(uint32_t now,
                                      const MetaSense::CANBus::Stats& canStats)
{
    const bool busFresh = (canStats.lastRxMs != 0U) &&
                          (elapsedMsSafe(now, canStats.lastRxMs) <= CAN_TEMP_TIMEOUT_MS);
    const bool counterAdvancing = (canStats.rxFrames > s_canStartRxFrames) ||
                                   (canStats.rxLeafFrames > s_canStartLeafFrames) ||
                                   (canStats.rx1daFrames > s_canStart1daFrames);
    const bool hasAnyActivity = busFresh &&
                                ((canStats.rxFrames > 0U) ||
                                 (canStats.rxLeafFrames > 0U) ||
                                 (canStats.last1daMs != 0U));

    if (counterAdvancing) {
        s_canStartRxFrames = canStats.rxFrames;
        s_canStartLeafFrames = canStats.rxLeafFrames;
        s_canStart1daFrames = canStats.rx1daFrames;
    }

    if (!busFresh) {
        s_canStartReadyLatched = false;
        return false;
    }

    if (counterAdvancing || s_canStartReadyLatched || hasAnyActivity) {
        s_canStartReadyLatched = true;
        return true;
    }

    return false;
}

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

        Serial0.printf("[Input] Load-cell ADC source ready (NAU7802 @ 0x2A, ldo=%d, cal=%d)\n",
                       ldoConfigured ? 1 : 0,
                       internalCalOk ? 1 : 0);
    } else {
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

    const char* hwState = MetaSense::HardwareOutputStateMachine::stateName();
    const bool inInitOrStart = (hwState != nullptr) &&
                               ((strcmp(hwState, "INIT") == 0) ||
                                (strcmp(hwState, "START") == 0));
    if (inInitOrStart && (nowMs - s_initCanTxLastMs) >= 100U) {
        s_initCanTxLastMs = nowMs;
        (void)MetaSense::Input::sendLeafTorqueCommand1d4FinalZero();
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
        // [PRODUCTION DIAGNOSTIC] Commented to test if CAN monitor updates were blocking WiFi
        // leafCanRpmMonitor = leafFb.rpm;
        // lastCanRpmMonitorUpdate = leafFb.rpm_update_ms;
        lastCanRpmFrameMs = leafFb.rpm_update_ms;
#if !METASENSE_FORCE_TACHO_RPM_SOURCE
        MetaSense::Input::updateCanRpm(leafFb.rpm);
#endif
        lastCanLeafAnyUpdate = leafFb.rpm_update_ms;

        if (kLeafCanHandshakeOnFirst1da && !s_leafHandshakeSent && !s_leafHandshakeArmed) {
            s_leafHandshakeArmed = true;
            s_leafHandshakeSentCount = 0;
            s_leafHandshakeAttemptCount = 0;
            s_leafHandshakeStartMs = nowMs;
            s_leafHandshakeLastAttemptMs = 0;
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

    static uint32_t lastDashboardMs = 0;
    static uint32_t lastIPCacheMs = 0;
    static String cachedIP = "0.0.0.0";
    static int cachedRSSI = 0;
    const uint32_t now = millis();
    
    // Check if we have WebSocket clients connected
    const uint32_t clientCount = MetaSense::WebSocketServer::socket().count();
    
    // Dashboard updates at 50ms (20 Hz) when browser is connected for smooth display without WiFi overload
    // Reduces to 100ms when idle (no clients) to save power
    const uint32_t dashboardCadenceMs = (clientCount > 0) ? 50U : 100U;    // 50ms for stable updates

    // === LIGHTWEIGHT DATA: Essential telemetry ===
    // Browser expects "data" message type with ALL these fields
    if (now - lastDashboardMs >= dashboardCadenceMs) {
        lastDashboardMs = now;
        
        // Use static buffer for JSON to avoid String() float conversion issues
        static char jsonBuffer[2600];  // Increased to 2600 for 0x11A fields
        int pos = 0;
        const auto& leafFb = MetaSense::CANBus::feedback();
        
        // FIX: Use stats data that was captured ATOMICALLY at frame reception
        // last1daData[] and last1daWireCrcCalc are always synchronized (set together in CANBus handler)
        // This guarantees CRC RX, Calc, and OK are from the SAME frame reception event
        const auto& canStats = MetaSense::CANBus::stats();
        const uint8_t leaf1daCrcRx = canStats.last1daData[7];  // CRC RX from last reception
        const uint8_t leaf1daCrcCalc = canStats.last1daWireCrcCalc;  // CRC Calc from same reception
        const int leaf1daCrcOk = (canStats.last1daWireCrcOk > 0) ? 1 : 0;  // Match result from same reception
        
        // Capture 0x1D4 TX CRC atomically from the cached frame data
        // This ensures CRC RX, Calc, and OK are from the SAME frame being sent
        const uint8_t leaf1d4CrcRx = s_leaf1d4PayloadCachedFrameData[7];  // CRC TX from cached frame
        const uint8_t leaf1d4CrcCalc = computeLeaf1d4CrcConformant(s_leaf1d4PayloadCachedFrameData);  // CRC Calc from same frame
        const int leaf1d4CrcOk = (leaf1d4CrcRx == leaf1d4CrcCalc) ? 1 : 0;  // Match result from same frame
        
        // Build JSON using snprintf for robust numeric formatting
        pos += snprintf(jsonBuffer + pos, sizeof(jsonBuffer) - pos,
            "{\"type\":\"data\","
            "\"rpm\":%.1f,"
            "\"rpm_error\":0,"
            "\"drum_rpm\":%.1f,"
            "\"kw\":%.2f,"
            "\"peakKW\":%.2f,"
            "\"peakKW_RPM\":%.0f,"
            "\"torque\":%.2f,"
            "\"brakeTorque\":%.2f,"
            "\"torque_measured\":%.2f,"
            "\"load_kg\":%.2f,"
            "\"throttle_pct\":%.1f,"
            "\"peakTorque\":%.2f,"
            "\"peakTorque_RPM\":%.0f,"
            "\"e_torque\":%.2f,"
            "\"energy\":%.2f,"
            "\"energy_active\":%d,"
            "\"rel_humidity\":%.1f,"
            "\"ratio_confidence\":0,"
            "\"rpm_target\":%.0f,"
            "\"can_fallback\":0,"
            "\"rpm_source_active\":\"leafrpm\","
            "\"kp_source\":\"firmware\","
            "\"kp_live\":0,"
            "\"ki_live\":0,"
            "\"egt_hot\":%.1f,"
            "\"egt_status\":%d,"
            "\"egt_ready\":%d,"
            "\"pressure\":%.1f,"
            "\"ambient_temp\":%.1f,"
            "\"air_density\":%.3f,"
            "\"climate_cf\":%.3f,"
            "\"dyno_mode\":\"%s\","
            "\"inv_ready\":%d,"
            "\"sw_active\":%d,"
            "\"load_raw\":0,"
            "\"nau_ready\":0,"
            "\"recording\":%d,"
            "\"lambda\":%.2f,"
            "\"massflow_m3h\":%.2f,"
            "\"leaf_rpm\":%.0f,"
            "\"leaf_torque\":%.2f,"
            "\"leaf_torque_demand\":%.2f,"
            "\"leaf_torque_demand_manual\":%.2f,"
            "\"leaf_torque_mode\":\"%s\","
            "\"leaf_1da_input_v\":%.1f,"
            "\"leaf_1da_torque_nm\":%.2f,"
            "\"leaf_1da_rpm\":%.0f,"
            "\"leaf_1da_clock\":%d,"
            "\"leaf_1da_err\":0,"
            "\"leaf_1da_inv_fault_map\":%d,"
            "\"leaf_1da_inv_blinky\":0,"
            "\"leaf_1da_inv_fault_can_timeout\":0,"
            "\"leaf_1da_crc\":%d,"
            "\"leaf_1da_crc_calc\":%d,"
            "\"leaf_1da_crc_wire_calc\":%d,"
            "\"leaf_1da_crc_ok\":%d,"
            "\"leaf_1da_crc_wire_ok\":%d,"
            "\"leaf_1da_crc_wire_trusted\":%d,"
            "\"leaf_1da_crc_ok_frames\":%lu,"
            "\"leaf_1da_crc_bad_frames\":%lu,"
            "\"leaf_1da_crc_wire_ok_frames\":%lu,"
            "\"leaf_1da_crc_wire_bad_frames\":%lu,"
            "\"leaf_1da_raw_b0b7\":\"%02X %02X %02X %02X %02X %02X %02X %02X\","
            "\"leaf_1da_inv_temp\":%.1f,"
            "\"leaf_1da_stator_temp\":%.1f,"
            "\"leaf_coolant_temp\":%.1f,"
            "\"leaf_1da_inv_status_bit\":%d,"
            "\"leaf_ready\":%d,"
            "\"hw_precharge\":%d,"
            "\"hw_rb_plus\":%d,"
            "\"hw_rb_minus\":%d,"
            "\"hw_ssr\":%d,"
            "\"hw_state\":\"%s\","
            "\"leaf_1d4_tx_frames\":%d,"
            "\"leaf_1d4_tx_age_ms\":%lu,"
            "\"leaf_1d4_tx_torque_nm\":%.2f,"
            "\"leaf_1d4_tx_torque_raw\":%d,"
            "\"leaf_1d4_tx_hv_status\":%d,"
            "\"leaf_1d4_tx_relay_plus\":%d,"
            "\"leaf_1d4_tx_charge_status\":%d,"
            "\"leaf_1d4_tx_clock\":%d,"
            "\"leaf_1d4_tx_crc\":%d,"
            "\"leaf_1d4_tx_crc_calc\":%d,"
            "\"leaf_1d4_tx_crc_ok\":%d,"
            "\"leaf_1d4_tx_ring_source_age\":%d,"
            "\"leaf_1d4_tx_ring_fallback_total\":%lu,"
            "\"leaf_1d4_tx_raw_b0b7\":\"%02X %02X %02X %02X %02X %02X %02X %02X\","
            "\"leaf_11a_frames\":%d,"
            "\"leaf_11a_age_ms\":%lu,"
            "\"leaf_11a_tx_frames\":%d,"
            "\"leaf_11a_tx_age_ms\":%lu,"
            "\"leaf_11a_gear\":%d,"
            "\"leaf_11a_car_onoff\":%d,"
            "\"leaf_11a_eco\":%d,"
            "\"leaf_11a_heartbeat\":%d,"
            "\"leaf_11a_mux\":%d,"
            "\"leaf_11a_button\":%d,"
            "\"leaf_11a_raw_b0b7\":\"%02X %02X %02X %02X %02X %02X %02X %02X\","
            "\"leaf_11a_tx_raw_b0b7\":\"%02X %02X %02X %02X %02X %02X %02X %02X\""
            "}",
            data.rpm,
            data.drumRpm,
            data.kw,
            data.peakKW,
            data.peakKW_RPM,
            data.torqueNm,
            data.brakeTorqueNm,
            data.torqueNm,
            data.loadKg,
            data.throttlePercent,
            data.peakTorque,
            data.peakTorque_RPM,
            data.eTorque,
            data.energyMJ,
            isRecording ? 1 : 0,
            data.humidity,
            data.rpmTarget,
            data.egtHotC,
            (int)data.egtStatus,
            data.egtReady ? 1 : 0,
            data.pressureHpa,
            data.ambientC,
            data.airDensity,
            data.climateCF,
            MetaSense::toString(data.mode),
            data.leaf_invReady ? 1 : 0,
            data.swActive ? 1 : 0,
            isRecording ? 1 : 0,
            data.lambdaValue,
            data.massflowM3h,
            data.leaf_rpm,
            data.leaf_torqueNm,
            data.leaf_torqueDemandNm,
            MetaSense::Input::getLeafUiTorqueDemandNm(),
            MetaSense::Input::getLeafManualTorqueMode() ? "manual" : "auto",
            leafFb.input_voltage,
            leafFb.torque_nm,
            data.leaf_rpm,  // leaf_1da_rpm
            (int)leafFb.mg_clock,  // leaf_1da_clock
            (int)leafFb.inv_fault_map,  // leaf_1da_inv_fault_map
            (int)leaf1daCrcRx,  // leaf_1da_crc (CRC RX) - from atomic frame
            (int)leaf1daCrcCalc,  // leaf_1da_crc_calc - from atomic frame
            (int)canStats.last1daWireCrcCalc,  // leaf_1da_crc_wire_calc - calculated at reception
            leaf1daCrcOk,  // leaf_1da_crc_ok - from atomic frame
            (canStats.last1daWireCrcOk > 0) ? 1 : 0,  // leaf_1da_crc_wire_ok - wire CRC status
            (canStats.last1daWireCrcOk > 0) ? 1 : 0,  // leaf_1da_crc_wire_trusted - same as wire_ok
            canStats.rx1daWireCrcOkFrames,  // leaf_1da_crc_ok_frames - total accepted frames
            canStats.rx1daWireCrcBadFrames,  // leaf_1da_crc_bad_frames - total rejected frames
            canStats.rx1daWireCrcOkFrames,  // leaf_1da_crc_wire_ok_frames - wire CRC good count
            canStats.rx1daWireCrcBadFrames,  // leaf_1da_crc_wire_bad_frames - wire CRC bad count
            canStats.last1daData[0],  // Use same atomic frame data as CRC
            canStats.last1daData[1],
            canStats.last1daData[2],
            canStats.last1daData[3],
            canStats.last1daData[4],
            canStats.last1daData[5],
            canStats.last1daData[6],
            canStats.last1daData[7],
            data.leaf_invTempC,  // leaf_inv_temp
            data.leaf_statorTempC,  // leaf_stator_temp
            data.leaf_coolantTempC,  // leaf_coolant_temp
            (int)leafFb.inv_status_bit,  // leaf_1da_inv_status_bit (Inv_StatusBit: 1=ready, 0=not ready)
            leafFb.ready ? 1 : 0,  // leaf_ready
            MetaSense::HardwareOutputStateMachine::isPrechargeActive() ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::isRbPlusActive() ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::isRbMinusActive() ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::isSsrActive() ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::stateName(),  // hw_state (INIT/START/IDLE/MOTOR/DYNO)
            (int)MetaSense::CANBus::stats().tx1d4Frames,  // leaf_1d4_tx_frames
            now - MetaSense::CANBus::stats().last1d4TxMs,  // leaf_1d4_tx_age_ms
            s_leaf1d4PayloadTorqueNm,  // leaf_1d4_tx_torque_nm
            (int)s_leaf1d4PayloadTorqueRaw,  // leaf_1d4_tx_torque_raw
            s_leaf1d4PayloadHvStatus ? 1 : 0,  // leaf_1d4_tx_hv_status
            s_leaf1d4PayloadRelayPlus ? 1 : 0,  // leaf_1d4_tx_relay_plus
            (int)s_leaf1d4PayloadChargeStatus,  // leaf_1d4_tx_charge_status
            (int)s_leaf1d4PayloadClock,  // leaf_1d4_tx_clock
            (int)leaf1d4CrcRx,  // leaf_1d4_tx_crc (from actual cached frame)
            (int)leaf1d4CrcCalc,  // leaf_1d4_tx_crc_calc (calculated from cached frame)
            leaf1d4CrcOk,  // leaf_1d4_tx_crc_ok
            (int)s_leaf1d4TxRingSourceAge,  // leaf_1d4_tx_ring_source_age
            s_leaf1d4TxRingFallbackCount,  // leaf_1d4_tx_ring_fallback_total
            s_leaf1d4PayloadCachedFrameData[0],  // leaf_1d4_tx_raw_b0
            s_leaf1d4PayloadCachedFrameData[1],  // leaf_1d4_tx_raw_b1
            s_leaf1d4PayloadCachedFrameData[2],  // leaf_1d4_tx_raw_b2
            s_leaf1d4PayloadCachedFrameData[3],  // leaf_1d4_tx_raw_b3
            s_leaf1d4PayloadCachedFrameData[4],  // leaf_1d4_tx_raw_b4
            s_leaf1d4PayloadCachedFrameData[5],  // leaf_1d4_tx_raw_b5
            s_leaf1d4PayloadCachedFrameData[6],  // leaf_1d4_tx_raw_b6
            s_leaf1d4PayloadCachedFrameData[7],  // leaf_1d4_tx_raw_b7
            // 0x11A RX frame data
            (int)MetaSense::CANBus::stats().rx11aFrames,  // leaf_11a_frames
            now - MetaSense::CANBus::stats().last11aMs,  // leaf_11a_age_ms
            (int)MetaSense::CANBus::stats().tx11aFrames,  // leaf_11a_tx_frames
            now - MetaSense::CANBus::stats().last11aTxMs,  // leaf_11a_tx_age_ms
            (int)((MetaSense::CANBus::stats().last11aData[0] >> 4U) & 0x0FU),  // leaf_11a_gear
            (int)((MetaSense::CANBus::stats().last11aData[1] >> 5U) & 0x07U),  // leaf_11a_car_onoff
            (int)((MetaSense::CANBus::stats().last11aData[1] >> 4U) & 0x01U),  // leaf_11a_eco
            (int)MetaSense::CANBus::stats().last11aData[3],  // leaf_11a_heartbeat
            (int)MetaSense::CANBus::stats().last11aData[6],  // leaf_11a_mux
            (int)MetaSense::CANBus::stats().last11aData[2],  // leaf_11a_button
            MetaSense::CANBus::stats().last11aData[0],  // leaf_11a_raw_b0
            MetaSense::CANBus::stats().last11aData[1],  // leaf_11a_raw_b1
            MetaSense::CANBus::stats().last11aData[2],  // leaf_11a_raw_b2
            MetaSense::CANBus::stats().last11aData[3],  // leaf_11a_raw_b3
            MetaSense::CANBus::stats().last11aData[4],  // leaf_11a_raw_b4
            MetaSense::CANBus::stats().last11aData[5],  // leaf_11a_raw_b5
            MetaSense::CANBus::stats().last11aData[6],  // leaf_11a_raw_b6
            MetaSense::CANBus::stats().last11aData[7],  // leaf_11a_raw_b7
            MetaSense::CANBus::stats().last11aTxData[0],  // leaf_11a_tx_raw_b0
            MetaSense::CANBus::stats().last11aTxData[1],  // leaf_11a_tx_raw_b1
            MetaSense::CANBus::stats().last11aTxData[2],  // leaf_11a_tx_raw_b2
            MetaSense::CANBus::stats().last11aTxData[3],  // leaf_11a_tx_raw_b3
            MetaSense::CANBus::stats().last11aTxData[4],  // leaf_11a_tx_raw_b4
            MetaSense::CANBus::stats().last11aTxData[5],  // leaf_11a_tx_raw_b5
            MetaSense::CANBus::stats().last11aTxData[6],  // leaf_11a_tx_raw_b6
            MetaSense::CANBus::stats().last11aTxData[7]   // leaf_11a_tx_raw_b7
        );
        
        wsock.textAll(jsonBuffer);
        // Note: All CAN monitor fields (leaf_1da_*) already included in dashboard JSON above
    }

    // === EXCEPTIONS ONLY: Send alerts for errors/faults via serial ===
    // Log warnings when conditions change
    static uint32_t lastExceptionCheckMs = 0;
    if ((now - lastExceptionCheckMs) > 5000) {  // Check every 5 seconds
        lastExceptionCheckMs = now;
        const auto& leafFb = MetaSense::CANBus::feedback();
        
        // Log fault conditions
        if (leafFb.inv_fault_map != 0) {
            Serial.printf("[WARNING] Inverter fault detected: 0x%02X\n", leafFb.inv_fault_map);
        }
        if (leafFb.mg_error_codes != 0) {
            Serial.printf("[WARNING] MG error codes: 0x%02X\n", leafFb.mg_error_codes);
        }
        if (data.leaf_invTempC > 80.0f) {
            Serial.printf("[WARNING] Inverter temperature high: %.1f°C\n", data.leaf_invTempC);
        }
        if (data.leaf_statorTempC > 150.0f) {
            Serial.printf("[WARNING] Stator temperature high: %.1f°C\n", data.leaf_statorTempC);
        }
        if (data.leaf_coolantTempC > 95.0f) {
            Serial.printf("[WARNING] Coolant temperature high: %.1f°C\n", data.leaf_coolantTempC);
        }
    }
}

void publishTelemetry()
{
    static uint32_t lastPublishedMs = 0;

    const uint32_t now = millis();
    // Send telemetry on a fixed 50ms cadence (20 Hz for stable smooth updates)
    if (now - lastPublishedMs < kWebSocketPublishPeriodMs) {
        return;
    }

    lastPublishedMs = now;
    const MetaSense::Telemetry telemetry = MetaSense::RunStorage::latest();
    // Telemetry sending: smooth display without overwhelming WebSocket queue
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

void setLeafUiTorqueDemandNm(float torqueNm)
{
    setLeafUiTorqueDemandNmInternal(torqueNm);
}

float getLeafUiTorqueDemandNm()
{
    return getLeafUiTorqueDemandNmInternal();
}

void setLeafManualTorqueMode(bool manualMode)
{
    s_leafManualTorqueMode = manualMode;
}

bool getLeafManualTorqueMode()
{
    return s_leafManualTorqueMode;
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
    if (!MetaSense::Globals::kVcuSwitch) {
        return "bench_forced";
    }

    if (!tele.vcuReady) {
        return "not_ready";
    }

    const uint32_t now = millis();
    const LeafInvFeedback& fb = MetaSense::CANBus::feedback();
    const MetaSense::CANBus::Stats& canStats = MetaSense::CANBus::stats();
    const bool nativeStatusFresh = (fb.status_update_ms != 0U) &&
                                   (elapsedMsSafe(now, fb.status_update_ms) < CAN_TEMP_TIMEOUT_MS);
    const bool rpmFreshForFallback = is1daWireCrcTrustedForFallback(canStats, now);
    const bool id55aFreshForFallback = (canStats.last55aMs != 0U) &&
                                       (elapsedMsSafe(now, canStats.last55aMs) <= CAN_RX_TARGET_MAX_AGE_MS);

    if (nativeStatusFresh) {
        return "native_1d4";
    }

    if (id55aFreshForFallback) {
        return "fallback_55a";
    }

    if (rpmFreshForFallback) {
        return "fallback_rpm_1da";
    }

    return "gpio_rb_plus";
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

    // Allow first sample and stale-source reacquisition before applying jump filter.
    const bool hasRecentCanSample = (lastCanRpmUpdate != 0U) &&
                                    ((now - lastCanRpmUpdate) < CAN_RPM_TIMEOUT_MS);
    if (hasRecentCanSample && fabs(rpm - lastCanRpm) > CAN_MAX_JUMP)
        return;

    if (rpm >= 0 && rpm < 20000) {
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

struct Leaf11aFrameFields {
    uint8_t joystickGearPosition = 0U;
    uint8_t ecoSelected = 0U;
    uint8_t carOnOffStatus = 0U;
    uint8_t steeringWheelButton = 0U;
    uint8_t heartbeatVcm = 0x55U;
    uint8_t unknown11a4 = 0U;
    uint8_t multiplexor = 0U;
    uint8_t startupData = 0U;
};

Leaf11aFrameFields decodeLeaf11aFields(const uint8_t* data, uint8_t len)
{
    Leaf11aFrameFields fields;
    if (data == nullptr || len < 8U) {
        return fields;
    }

    // DBC layout: 0x11A uses the low nibble of byte 0 for gear,
    // byte 1 bits 4..6 for ECO + CarOnOff, and bytes 2/3/4/6/7 for the
    // remaining fixed, muxed, and startup fields.
    fields.joystickGearPosition = static_cast<uint8_t>((data[0] >> 4U) & 0x0FU); // 4|4@1+
    fields.ecoSelected = static_cast<uint8_t>((data[1] >> 4U) & 0x01U);           // 12|1@1+
    fields.carOnOffStatus = static_cast<uint8_t>((data[1] >> 5U) & 0x07U);        // 13|3@1+
    fields.steeringWheelButton = data[2];                                          // 16|8@1+
    fields.heartbeatVcm = data[3];                                                 // 24|8@1+
    fields.unknown11a4 = data[4];                                                  // 32|8@1+
    fields.multiplexor = data[6];                                                  // 48|8@1+
    fields.startupData = data[7];                                                  // 56|8@1+
    return fields;
}

void encodeLeaf11aFrame(const Leaf11aFrameFields& fields, uint8_t (&out)[8], const uint8_t (&base)[8])
{
    memcpy(out, base, sizeof(out));

    // Keep the runtime write-back aligned with the confirmed DBC map.
    out[0] = static_cast<uint8_t>((out[0] & 0x0FU) | ((fields.joystickGearPosition & 0x0FU) << 4U));
    out[1] = static_cast<uint8_t>((out[1] & 0x0FU) |
                                  ((fields.ecoSelected & 0x01U) << 4U) |
                                  ((fields.carOnOffStatus & 0x07U) << 5U));
    out[2] = fields.steeringWheelButton;
    out[3] = fields.heartbeatVcm;
    out[4] = fields.unknown11a4;
    out[6] = fields.multiplexor;
    out[7] = fields.startupData;
}

bool sendLeafKeepAlive11a(uint32_t nowMs, bool forceImmediate = false, uint8_t muxOverride = 0xFFU)
{
    if (!kLeaf11aTxEnabled) {
        return false;
    }
    if (!MetaSense::CANBus::isReady()) {
        return false;
    }
    if (!forceImmediate && lastLeaf11aTxMs != 0U && (nowMs - lastLeaf11aTxMs) < kLeaf11aTxPeriodMs) {
        return false;
    }

    const MetaSense::CANBus::Stats& canStats = MetaSense::CANBus::stats();
    if (!s_leaf11aMuxTemplatesLocked && canStats.last11aLen >= 8U &&
        canStats.rx11aFrames != s_leaf11aMuxTemplateLastRxFrames) {
        const uint8_t rxMuxSel = static_cast<uint8_t>(canStats.last11aData[6] & 0x03U);
        const uint8_t rxMuxBit = static_cast<uint8_t>(1U << rxMuxSel);
        // While unlocked, always refresh the active slot from live TVCU RX.
        memcpy(s_leaf11aMuxTemplateBySel[rxMuxSel], canStats.last11aData, 8U);
        s_leaf11aMuxSeenMask |= rxMuxBit;
        s_leaf11aMuxRxSeenMask |= rxMuxBit;
        if (s_leaf11aMuxRxSeenMask == 0x0FU) {
            s_leaf11aMuxTemplatesLocked = true;
        }
        s_leaf11aMuxTemplateLastRxFrames = canStats.rx11aFrames;
    }

    if (!s_leaf11aMuxTemplatesLocked && s_leaf11aMuxSeenMask == 0U) {
        return false;
    }

    const uint8_t muxSel = (muxOverride <= 3U)
        ? static_cast<uint8_t>(muxOverride & 0x03U)
        : static_cast<uint8_t>(s_leaf11aMuxTxSeq & 0x03U);
    if ((s_leaf11aMuxSeenMask & static_cast<uint8_t>(1U << muxSel)) == 0U) {
        return false;
    }

    uint8_t templateData[8] = {0U};
    memcpy(templateData, s_leaf11aMuxTemplateBySel[muxSel], sizeof(templateData));

    Leaf11aFrameFields fields = decodeLeaf11aFields(templateData, sizeof(templateData));
    fields.multiplexor = muxSel;

    const uint8_t probeGear = static_cast<uint8_t>(METASENSE_LEAF_11A_FORCE_GEAR & 0x0FU);
    const uint8_t probeCarOnOff = static_cast<uint8_t>(METASENSE_LEAF_11A_FORCE_CARONOFF & 0x07U);
    fields.joystickGearPosition = static_cast<uint8_t>(probeGear & 0x0FU);
    fields.carOnOffStatus = static_cast<uint8_t>(probeCarOnOff & 0x07U);
    uint8_t txData[8] = {0U};
    encodeLeaf11aFrame(fields, txData, templateData);

    const bool sent = MetaSense::CANBus::send(0x11AU, txData, sizeof(txData));
    if (sent) {
        lastLeaf11aTxMs = nowMs;
        if (muxOverride <= 3U) {
            s_leaf11aMuxTxSeq = static_cast<uint8_t>((muxSel + 1U) & 0x03U);
        } else {
            s_leaf11aMuxTxSeq = static_cast<uint8_t>((s_leaf11aMuxTxSeq + 1U) & 0x03U);
        }
    }
    return sent;
}

void initLeaf11aBootstrapTemplates()
{
    for (uint8_t muxSel = 0U; muxSel < 4U; ++muxSel) {
        uint8_t* slot = s_leaf11aMuxTemplateBySel[muxSel];
        slot[0] = METASENSE_LEAF_11A_TEMPLATE_B0;
        slot[1] = METASENSE_LEAF_11A_TEMPLATE_B1;
        slot[2] = METASENSE_LEAF_11A_TEMPLATE_B2;
        slot[3] = METASENSE_LEAF_11A_TEMPLATE_B3;
        slot[4] = METASENSE_LEAF_11A_TEMPLATE_B4;
        slot[5] = METASENSE_LEAF_11A_TEMPLATE_B5;
        slot[6] = muxSel;
        switch (muxSel) {
        case 0U: slot[7] = METASENSE_LEAF_11A_TEMPLATE_M0_B7; break;
        case 1U: slot[7] = METASENSE_LEAF_11A_TEMPLATE_M1_B7; break;
        case 2U: slot[7] = METASENSE_LEAF_11A_TEMPLATE_M2_B7; break;
        default: slot[7] = METASENSE_LEAF_11A_TEMPLATE_M3_B7; break;
        }
    }

    // Bootstrap is a fallback seed. Keep learning enabled so live TVCU frames
    // can replace templates slot-by-slot and then lock after all 4 are seen.
    s_leaf11aMuxSeenMask = 0x0FU;
    s_leaf11aMuxRxSeenMask = 0U;
    s_leaf11aMuxTemplatesLocked = false;
}

bool sendLeafTorqueCommand1d4(float torqueDemandNm,
                              bool readyBit,
                              bool hvOkBit,
                              bool brakeBit,
                              bool gearDriveBit)
{
    (void)readyBit;
    (void)brakeBit;
    (void)gearDriveBit;

    const uint32_t nowMs = millis();
    const float uiTorqueDemandNm = getLeafUiTorqueDemandNmInternal();
    const bool manualCommandActive = s_leafCanPartnerSeen && !s_leafSimFeedbackActive;
    const bool startupReadyState = readyBit && hvOkBit && s_leafCanPartnerSeen;
    // Once partner comms are established, do not let ready-bit jitter zero out
    // manual control frames. A zero UI demand still sends zero naturally.
    const float effectiveTorqueDemandNm = manualCommandActive
        ? uiTorqueDemandNm
        : (startupReadyState ? torqueDemandNm : 0.0f);
    const float torqueClamped = constrain(effectiveTorqueDemandNm, -512.0f, 511.75f);

    const uint8_t hcmClock = static_cast<uint8_t>(s_leaf1d4RollingCounter & 0x03U);

    Leaf1d4FrameFields fields;
    fields.torqueDemandNm = torqueClamped;
    fields.torqueDemandRaw = encodeLeaf1d4TorqueRaw(torqueClamped);
    fields.hvStatus = hvOkBit;
    fields.rbPlus = vcuDebugRPlus || MetaSense::HardwareOutputStateMachine::isRbPlusCommandedActive();
    fields.chargeStatus = METASENSE_LEAF_1D4_TEMPLATE_B6;
    fields.cmdClock = hcmClock;

    // Prefer the newest live 0x1D4 sniff frame as TX base so non-CRC semantics
    // stay aligned with the inverter's accepted family. Fallback to template.
    uint8_t data[8] = {
        METASENSE_LEAF_1D4_TEMPLATE_B0,
        METASENSE_LEAF_1D4_TEMPLATE_B1,
        METASENSE_LEAF_1D4_TEMPLATE_B2,
        METASENSE_LEAF_1D4_TEMPLATE_B3,
        METASENSE_LEAF_1D4_TEMPLATE_B4,
        METASENSE_LEAF_1D4_TEMPLATE_B5,
        METASENSE_LEAF_1D4_TEMPLATE_B6,
        METASENSE_LEAF_1D4_TEMPLATE_B7
    };
    uint8_t ringBaseData[8] = {0U};
    uint8_t ringBaseLen = 0U;
    bool usedRingBase = false;
    if (MetaSense::CANBus::get1d4RingFrame(0U, ringBaseData, &ringBaseLen) && ringBaseLen >= 8U) {
        memcpy(data, ringBaseData, 8U);
        usedRingBase = true;
    }

    patchLeaf1d4TorqueFieldMotorola23_12(data, fields.torqueDemandRaw);
    setIntelUnsigned(data, 8U, 34U, 1U, fields.hvStatus ? 1U : 0U);
    setIntelUnsigned(data, 8U, 46U, 1U, fields.rbPlus ? 1U : 0U);
    setIntelUnsigned(data, 8U, 38U, 2U, static_cast<uint32_t>(fields.cmdClock & 0x03U));
    // Keep fixed template header bytes explicit only for template fallback mode.
    if (!usedRingBase) {
        data[0] = METASENSE_LEAF_1D4_TEMPLATE_B0;
        data[1] = METASENSE_LEAF_1D4_TEMPLATE_B1;
    }
    data[7] = computeLeaf1d4CrcConformant(data);

    const int16_t torqueRaw = fields.torqueDemandRaw;

    // Preserve the generated payload snapshot even if bus TX fails, so monitor/debug
    // comparisons show the intended command bytes instead of stale zeros.
    memcpy(s_leafLast1d4TxData, data, sizeof(s_leafLast1d4TxData));
    s_leafLast1d4TxLen = sizeof(data);

    const bool sent = MetaSense::CANBus::send(0x1D4U, data, sizeof(data));
    if (sent) {
        const uint8_t crcWire = data[7];
        const uint8_t crc1daStyle = MetaSense::LeafCRC::computeExact1daWireCrc(0xD4U, data);
        const uint8_t crcResidue = MetaSense::LeafCRC::computeApprovedInverterCrc(0xD4U, data);
        const uint8_t crc1d4Like = MetaSense::LeafCRC::computeExact1d4LikeCrc(0xD4U, data);
        const uint8_t clockB4 = static_cast<uint8_t>((data[4] >> 6U) & 0x03U);
        const uint8_t clockB6 = static_cast<uint8_t>(data[6] & 0x03U);
        static uint32_t s_leaf1d4TxDiagLastMs = 0U;

        s_leaf1d4RollingCounter = static_cast<uint8_t>((s_leaf1d4RollingCounter + 1U) & 0x03U);
        s_leafLast1d4TxMs = nowMs;
        s_leafLastSentTorqueNm = torqueClamped;
        s_leafLastSentTorqueMs = nowMs;
        s_leaf1d4PayloadTorqueNm = torqueClamped;
        s_leaf1d4PayloadTorqueRaw = torqueRaw;
        s_leaf1d4PayloadHvStatus = fields.hvStatus;
        s_leaf1d4PayloadRelayPlus = fields.rbPlus;
        s_leaf1d4PayloadChargeStatus = data[6];
        s_leaf1d4PayloadClock = hcmClock;
        s_leaf1d4PayloadCrc = data[7];
        s_leaf1d4PayloadCrcCalc = computeLeaf1d4CrcConformant(data);
        s_leaf1d4PayloadCrcOk = (s_leaf1d4PayloadCrc == s_leaf1d4PayloadCrcCalc) ? 1 : 0;
        s_leaf1d4PayloadMs = nowMs;
    }

    return sent;
}

bool sendLeafTorqueCommand1d4AndKeepAlive11a(float torqueDemandNm,
                                            bool readyBit,
                                            bool hvOkBit,
                                            bool brakeBit,
                                            bool gearDriveBit,
                                            uint32_t nowMs,
                                            bool force11aImmediate)
{
    const bool sent1d4 = sendLeafTorqueCommand1d4(torqueDemandNm,
                                                 readyBit,
                                                 hvOkBit,
                                                 brakeBit,
                                                 gearDriveBit);
    if (!sent1d4) {
        return false;
    }

    const bool sent11a = sendLeafKeepAlive11a(nowMs,
                                             force11aImmediate,
                                             static_cast<uint8_t>(s_leaf1d4PayloadClock & 0x03U));
    return sent11a || sent1d4;
}

bool sendLeafTorqueCommand1d4FinalZero()
{
    return sendLeafTorqueCommand1d4(0.0f, false, false, false, false);
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

float computeTorqueStepSequence(uint32_t nowMs, bool torqueGateArmed, bool inverterStatusClear)
{
    (void)nowMs;
    (void)torqueGateArmed;
    (void)inverterStatusClear;
    return 0.0f;
}

// ============================================================================
// 0x1D4 Payload State Machine (100ms update cadence, independent of TX rate)
// ============================================================================
// Generates 0x1D4 frame payload from template + selected torque source
// This runs every 100ms and caches the complete frame for 10ms TX loop to send
static void updateLeaf1d4PayloadStateMachine(uint32_t nowMs)
{
    // Update only every 100ms
    if ((nowMs - s_leaf1d4PayloadStateLastUpdateMs) < LEAF_1D4_TORQUE_PAYLOAD_UPDATE_PERIOD_MS) {
        return;  // Not time yet
    }
    s_leaf1d4PayloadStateLastUpdateMs = nowMs;

    // Select torque source: automatic (PI control) or manual (user command)
    const float selectedTorqueNm = s_leaf1d4TorqueSourceAutomatic
        ? s_leafTxPacerTorqueNm      // Automatic: from PI control
        : getLeafUiTorqueDemandNmInternal();  // Manual: from UI/user input
    
    const float torqueClamped = constrain(selectedTorqueNm, -512.0f, 511.75f);

    // Increment rolling counter for frame sequence tracking
    const uint8_t hcmClock = static_cast<uint8_t>(s_leaf1d4RollingCounter & 0x03U);

    // Build frame from template base (initialized during CAN setup)
    uint8_t data[8] = {
        METASENSE_LEAF_1D4_TEMPLATE_B0,
        METASENSE_LEAF_1D4_TEMPLATE_B1,
        METASENSE_LEAF_1D4_TEMPLATE_B2,
        METASENSE_LEAF_1D4_TEMPLATE_B3,
        METASENSE_LEAF_1D4_TEMPLATE_B4,
        METASENSE_LEAF_1D4_TEMPLATE_B5,
        METASENSE_LEAF_1D4_TEMPLATE_B6,
        METASENSE_LEAF_1D4_TEMPLATE_B7
    };

    // Prefer live sniffed 0x1D4 frame as base to maintain inverter alignment
    uint8_t ringBaseData[8] = {0U};
    uint8_t ringBaseLen = 0U;
    bool usedRingBase = false;
    if (MetaSense::CANBus::get1d4RingFrame(0U, ringBaseData, &ringBaseLen) && ringBaseLen >= 8U) {
        memcpy(data, ringBaseData, 8U);
        usedRingBase = true;
    }

    // Encode selected torque value into frame
    const int16_t torqueRaw = encodeLeaf1d4TorqueRaw(torqueClamped);
    patchLeaf1d4TorqueFieldMotorola23_12(data, torqueRaw);
    
    // Update HV status bit (bit 34)
    const bool hvOkBit = s_leafTxPacerHvOkBit;
    setIntelUnsigned(data, 8U, 34U, 1U, hvOkBit ? 1U : 0U);
    
    // Update RB+ relay bit (bit 46)
    const bool rbPlusActive = vcuDebugRPlus || MetaSense::HardwareOutputStateMachine::isRbPlusCommandedActive();
    setIntelUnsigned(data, 8U, 46U, 1U, rbPlusActive ? 1U : 0U);
    
    // Note: Clock bits (38-39) and CRC will be updated by the 10ms TX task before sending
    // This ensures CRC and clock are fresh for EVERY transmission, not just every 100ms
    
    // Keep template header bytes only if not using live sniffed frame
    if (!usedRingBase) {
        data[0] = METASENSE_LEAF_1D4_TEMPLATE_B0;
        data[1] = METASENSE_LEAF_1D4_TEMPLATE_B1;
    }

    // Force bytes 5 and 6 to always use template values (static fields)
    // Byte 4 (counter) is updated dynamically every frame in leafTxPacerTask
    data[5] = METASENSE_LEAF_1D4_TEMPLATE_B5;  // 0x44 (charge status)
    data[6] = METASENSE_LEAF_1D4_TEMPLATE_B6;  // 0x01 (unknown field)

    // Cache the complete frame for 10ms TX loop (CRC/clock will be updated there)
    memcpy(s_leaf1d4PayloadCachedFrameData, data, sizeof(s_leaf1d4PayloadCachedFrameData));
    
    // Update payload metadata for telemetry/logging
    s_leaf1d4PayloadTorqueNm = torqueClamped;
    s_leaf1d4PayloadTorqueRaw = torqueRaw;
    s_leaf1d4PayloadHvStatus = hvOkBit;
    s_leaf1d4PayloadRelayPlus = rbPlusActive;
    s_leaf1d4PayloadChargeStatus = data[6];
    s_leaf1d4PayloadClock = hcmClock;
    s_leaf1d4PayloadCrc = data[7];
    s_leaf1d4PayloadCrcCalc = computeLeaf1d4CrcConformant(data);
    s_leaf1d4PayloadCrcOk = (s_leaf1d4PayloadCrc == s_leaf1d4PayloadCrcCalc) ? 1 : 0;
    s_leaf1d4PayloadMs = nowMs;
}

// Global counter for core-0 Serial output (tracks 0x1D4 frame transmissions)
static volatile uint32_t s_leaf1d4TxFrameCount = 0U;  // Incremented every 10ms frame transmission

void leafTxPacerTask(void* /*param*/)
{
    constexpr uint32_t kLeafTxPacerTickMs = CAN_TX_PERIOD_MS;  // 10ms
    TickType_t lastWake = xTaskGetTickCount();
    bool firstRun = true;  // Force state machine to run immediately on first TX
    
    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kLeafTxPacerTickMs));

        const uint32_t nowMs = millis();
        const bool leafTxChecklistActive = MetaSense::CANBus::isReady() &&
                                           (kLeafCanTxActive ||
                                            (kLeafCanHandshakeOnFirst1da && s_leafHandshakeSent));

        if (!s_leafTxPacerEnabled) {
            if (leafTxChecklistActive) {
                (void)sendLeafKeepAlive11a(nowMs);
            }
            continue;
        }

        // Force state machine to run on very first iteration to ensure proper frame initialization
        if (firstRun) {
            firstRun = false;
            s_leaf1d4PayloadStateLastUpdateMs = 0;  // Force immediate execution
        }

        // Update 0x1D4 payload state machine (runs every 100ms internally, check is cheap)
        // This generates and caches the complete frame every 100ms
        updateLeaf1d4PayloadStateMachine(nowMs);

        // Advance rolling counter BEFORE updating frame (0-3 cycle, changes every 10ms transmission)
        s_leaf1d4RollingCounter = static_cast<uint8_t>((s_leaf1d4RollingCounter + 1U) & 0x03U);

        // Update byte 4 with rolling counter - lookup table based on DBC HCM_CLOCK signal
        // Counter values 0,1,2,3 → upper nibbles 8,C,0,4 → byte values 0x87, 0xC7, 0x07, 0x47
        const uint8_t hcmClock = static_cast<uint8_t>(s_leaf1d4RollingCounter & 0x03U);
        static const uint8_t COUNTER_NIBBLE_MAP[] = {0x8U, 0xCU, 0x0U, 0x4U};
        s_leaf1d4PayloadCachedFrameData[4] = static_cast<uint8_t>((COUNTER_NIBBLE_MAP[hcmClock] << 4) | 0x7U);
        
        // Clear CRC field before recalculating (CRC always calculated with checksum = 0)
        s_leaf1d4PayloadCachedFrameData[7] = 0U;
        
        // Recalculate CRC after updating counter (fresh CRC every 10ms transmission)
        s_leaf1d4PayloadCachedFrameData[7] = computeLeaf1d4CrcConformant(s_leaf1d4PayloadCachedFrameData);
        
        // VERIFICATION: Validate our generated 0x1D4 frame using wire CRC algorithm
        // If this passes, it means the CRC algorithm is correct for inverter validation
        const uint8_t wireCrcRx = s_leaf1d4PayloadCachedFrameData[7];
        const uint8_t wireCrcCalc = MetaSense::LeafCRC::computeExact1d4LikeCrc(0xD4U, s_leaf1d4PayloadCachedFrameData);
        const bool frame1d4WireCrcOk = (wireCrcRx == wireCrcCalc);
        
        // Store verification result for monitoring (optional logging)
        s_leaf1d4PayloadCrcOk = frame1d4WireCrcOk ? 1 : 0;

        // Increment global counter for HEARTBEAT Serial output
        s_leaf1d4TxFrameCount++;

        // Send 0x1D4 frame with fresh CRC and clock every 10ms (maintains CAN bus presence)
        const bool sent1d4 = MetaSense::CANBus::send(0x1D4U, 
                                                      s_leaf1d4PayloadCachedFrameData, 
                                                      sizeof(s_leaf1d4PayloadCachedFrameData));
        
        // Update TX tracking
        if (sent1d4) {
            s_leafLast1d4TxMs = nowMs;
            s_leafLastSentTorqueNm = s_leaf1d4PayloadTorqueNm;
            s_leafLastSentTorqueMs = nowMs;
        }

        // Send 0x11A keep-alive frame every 10ms
        const bool sent11a = sendLeafKeepAlive11a(nowMs, true, hcmClock);

        // Log only when payload state machine updates (every 100ms)
        // Check if this is a logging frame by comparing update time with a logged time
        static uint32_t s_leafTxPacerLastLoggedMs = 0;
        if ((nowMs - s_leafTxPacerLastLoggedMs) >= LEAF_1D4_TORQUE_PAYLOAD_UPDATE_PERIOD_MS) {
            s_leafTxPacerLastLoggedMs = nowMs;
            // 0x120 shadow diagnostics removed - production uses 0x1D4 only
        }
    }
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
    setLeafUiTorqueDemandNmInternal(0.0f);
    s_leafTxPacerEnabled = true;  // DEBUG: Enable for testing
    s_leafTxPacerTorqueNm = 0.0f;
    s_leafTxPacerReadyBit = false;
    s_leafTxPacerHvOkBit = false;
    s_leafTxPacerBrakeBit = false;
    s_leafTxPacerGearDriveBit = true;
    s_leafLastSentTorqueNm = 0.0f;
    s_leafLastSentTorqueMs = 0;
    lastLeafTxMs = 0;
    lastLeaf11aTxMs = 0;
    s_leaf11aMuxTxSeq = 0U;
    s_leaf11aMuxSeenMask = 0U;
    s_leaf11aMuxRxSeenMask = 0U;
    s_leaf11aMuxTemplatesLocked = false;
    s_leaf11aMuxTemplateLastRxFrames = 0U;
    memset(s_leaf11aMuxTemplateBySel, 0, sizeof(s_leaf11aMuxTemplateBySel));
#if METASENSE_LEAF_11A_TEMPLATE_BOOTSTRAP_ENABLE
    initLeaf11aBootstrapTemplates();
#endif
    s_leaf1d4RollingCounter = 0U;
    s_leaf1d4ReplayIndex = 0U;
    s_leaf1d4PayloadStateLastUpdateMs = 0;
    memset(s_leaf1d4PayloadCachedFrameData, 0, sizeof(s_leaf1d4PayloadCachedFrameData));
    // Initialize cached frame with template
    s_leaf1d4PayloadCachedFrameData[0] = METASENSE_LEAF_1D4_TEMPLATE_B0;
    s_leaf1d4PayloadCachedFrameData[1] = METASENSE_LEAF_1D4_TEMPLATE_B1;
    s_leaf1d4PayloadCachedFrameData[2] = METASENSE_LEAF_1D4_TEMPLATE_B2;
    s_leaf1d4PayloadCachedFrameData[3] = METASENSE_LEAF_1D4_TEMPLATE_B3;
    s_leaf1d4PayloadCachedFrameData[4] = METASENSE_LEAF_1D4_TEMPLATE_B4;
    s_leaf1d4PayloadCachedFrameData[5] = METASENSE_LEAF_1D4_TEMPLATE_B5;
    s_leaf1d4PayloadCachedFrameData[6] = METASENSE_LEAF_1D4_TEMPLATE_B6;
    s_leaf1d4PayloadCachedFrameData[7] = METASENSE_LEAF_1D4_TEMPLATE_B7;
    memset(s_leafLast1d4TxData, 0, sizeof(s_leafLast1d4TxData));
    s_leafLast1d4TxLen = 0U;
    s_leafLast1d4TxMs = 0;
    s_leaf1d4PayloadTorqueNm = 0.0f;
    s_leaf1d4PayloadTorqueRaw = 0;
    s_leaf1d4PayloadHvStatus = false;
    s_leaf1d4PayloadRelayPlus = false;
    s_leaf1d4PayloadChargeStatus = 0U;
    s_leaf1d4PayloadClock = 0U;
    s_leaf1d4PayloadCrc = 0U;
    s_leaf1d4PayloadCrcCalc = 0U;
    s_leaf1d4PayloadCrcOk = -1;
    s_leaf1d4PayloadMs = 0;
    s_leaf1d4TxUsedRingBase = false;
    s_leaf1d4TxRingBaseLen = 0U;
    s_leaf1d4TxRingSourceAge = 0xFFU;
    s_leaf1d4TxRingFallbackCount = 0U;
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
    pinMode(MetaSense::Globals::kStartRequestPin, INPUT_PULLDOWN); // Start-request switch – active HIGH
    pinMode(MetaSense::Globals::kRbPlusInputPin, INPUT_PULLDOWN); // VCU ready – active HIGH
    prevSwState = (digitalRead(MetaSense::Globals::kRampSwitchPin) == HIGH);
    prevStartRequestState = (digitalRead(MetaSense::Globals::kStartRequestPin) == HIGH);
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

    if (leafTxPacerTaskHandle == nullptr) {
        // Prime the cached frame with a valid initial state before TX task starts
        uint8_t initData[8] = {
            METASENSE_LEAF_1D4_TEMPLATE_B0,
            METASENSE_LEAF_1D4_TEMPLATE_B1,
            METASENSE_LEAF_1D4_TEMPLATE_B2,
            METASENSE_LEAF_1D4_TEMPLATE_B3,
            METASENSE_LEAF_1D4_TEMPLATE_B4,
            METASENSE_LEAF_1D4_TEMPLATE_B5,
            METASENSE_LEAF_1D4_TEMPLATE_B6,
            METASENSE_LEAF_1D4_TEMPLATE_B7
        };
        // Initialize clock to 0, CRC will be computed fresh on first TX
        setIntelUnsigned(initData, 8U, 38U, 2U, 0U);
        initData[7] = computeLeaf1d4CrcConformant(initData);
        memcpy(s_leaf1d4PayloadCachedFrameData, initData, sizeof(s_leaf1d4PayloadCachedFrameData));
        s_leaf1d4RollingCounter = 0U;
        
        BaseType_t txTaskCreated = xTaskCreatePinnedToCore(
            leafTxPacerTask,
            "leafTxPacer",
            4096,
            nullptr,
            7,
            &leafTxPacerTaskHandle,
            1);
        if (txTaskCreated != pdPASS) {
            Serial.println("[Input] Failed to start leaf TX pacer task");
            Serial0.println("[Input] Failed to start leaf TX pacer task");
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

    const bool canBusReadyNow = MetaSense::CANBus::isReady();
    const bool leafTxStartAllowed = canBusReadyNow;
    const bool leafTxChecklistActive = leafTxStartAllowed &&
                                       (kLeafCanTxActive ||
                                        (kLeafCanHandshakeOnFirst1da && s_leafHandshakeSent));
    if (leafTxChecklistActive) {
        const bool sent = sendLeafTorqueCommand1d4FinalZero();
        // 0x120 shadow diagnostics removed
    }
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

    {
        const LeafInvFeedback& leafFbStatus = MetaSense::CANBus::feedback();
        const MetaSense::CANBus::Stats& canStatsStatus = MetaSense::CANBus::stats();
        const bool nativeStatusFresh = (leafFbStatus.status_update_ms != 0U) &&
                                       (elapsedMsSafe(now, leafFbStatus.status_update_ms) < CAN_TEMP_TIMEOUT_MS);
        const bool rpmFreshForFallback = is1daWireCrcTrustedForFallback(canStatsStatus, now);
        const bool id55aFreshForFallback = (canStatsStatus.last55aMs != 0U) &&
                                           (elapsedMsSafe(now, canStatsStatus.last55aMs) <= CAN_RX_TARGET_MAX_AGE_MS);

        // Universal readiness rule: when native 0x1D4 status is absent, derive
        // CAN readiness from incoming RX telemetry freshness. Prefer 0x55A
        // presence, with 0x1DA RPM freshness as secondary evidence.
        if (!nativeStatusFresh && (id55aFreshForFallback || rpmFreshForFallback)) {
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

    // VCU ready for bring-up follows the configured source:
    // - VCU_switch=1 uses the RB+ input pin (normal hardware path)
    // - VCU_switch=0 forces VCU-ready true so bench bring-up can proceed without a live inverter.
    const bool vcuReadyBase = MetaSense::Globals::kVcuSwitch
        ? (digitalRead(MetaSense::Globals::kRbPlusInputPin) == HIGH)
        : true;
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

    // --- START request switch (active HIGH, debounced) ---
    {
        const bool startReqNow = (digitalRead(MetaSense::Globals::kStartRequestPin) == HIGH);
        if (startReqNow != prevStartRequestState) {
            if ((now - startRequestDebounceMs) >= kStartRequestDebounceThresholdMs) {
                startRequestDebounceMs = now;
                prevStartRequestState = startReqNow;
                if (startReqNow) {
                    s_startSwitchOverridePending = true;
                    s_startSwitchOverrideRpm = kStartSwitchMotorRpm;
                    MetaSense::HardwareOutputStateMachine::requestMotorStartOverride(s_startSwitchOverrideRpm);
                    Serial.println("[START-BTN] motor-start override latched");
                    Serial0.println("[START-BTN] motor-start override latched");
                }
            }
        } else {
            startRequestDebounceMs = now;
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
    const bool canRpmAllowed = (!METASENSE_FORCE_TACHO_RPM_SOURCE);
    const MetaSense::CANBus::Stats& canStatsRpm = MetaSense::CANBus::stats();
    const bool canRpmFrameFresh = (canStatsRpm.last1daMs != 0U) &&
                                  (elapsedMsSafe(now, canStatsRpm.last1daMs) <= CAN_RX_TARGET_MAX_AGE_MS);
    const bool canRpmFrameWireCrcOk = is1daWireCrcTrustedForFallback(canStatsRpm, now);
    const bool canRpmFrameTrustworthy = canRpmFrameFresh && canRpmFrameWireCrcOk;
    bool canValid = canRpmAllowed && MetaSense::CANBus::isReady() && canRpmFrameTrustworthy;
    float rpmRaw = 0.0f;
    canFallbackActive = canRpmAllowed && !canValid;
    activeRpmFromCan = canValid;
    const float emotorRpmRaw = canRpmFrameTrustworthy ? leafCanRpmMonitor : canRpm;
    const float rpmRatio = (MetaSense::Settings::virtGearRatio > 0.01f)
        ? MetaSense::Settings::virtGearRatio
        : 1.45f;
    const float canEngineRpm = emotorRpmRaw * rpmRatio;
    if (canValid) {
        // Mandatory behavior: CAN source is unfiltered.
        rpmRaw = canEngineRpm;
        rpmFilt = canEngineRpm;
    } else {
        rpmRaw = tachoRpm;
        rpmFilt = lpFilter(rpmFilt, rpmRaw, alpha);
    }
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
    const bool canStatusClearForTorque = (leafFbNativeStatus.rpm_update_ms != 0U) &&
                                         (elapsedMsSafe(now, leafFbNativeStatus.rpm_update_ms) < CAN_TEMP_TIMEOUT_MS) &&
                                         (leafFbNativeStatus.mg_error_codes == 0U);
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

    if (s_startSwitchOverridePending) {
        tele.rpmTarget = s_startSwitchOverrideRpm;
        MetaSense::DynoStateMachine::setManualRpmTarget(s_startSwitchOverrideRpm);
        MetaSense::Settings::setRpmTarget(s_startSwitchOverrideRpm);
        s_startSwitchOverridePending = false;
    } else if (MetaSense::DynoStateMachine::isAutoRunActive()) {
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

    const bool torqueArmed = tele.rpmTarget > 400.0f;
    if (!torqueArmed) {
        torqueCmd = 0.0f;
    }

    if (!leafCmdFresh) {
        // Keep UI demand visible even if 0x120 TX path is stale/offline.
        tele.leaf_torqueDemandNm = torqueCmd;
    }

#if METASENSE_TORQUE_STEP_SEQUENCER_ENABLED
    if (torqueArmed && safe && canStatusClearForTorque) {
        torqueCmd = computeTorqueStepSequence(now, true, true);
    } else {
        torqueCmd = 0.0f;
        (void)computeTorqueStepSequence(now, false, canStatusClearForTorque);
    }
#else
    if (torqueArmed && !canStatusClearForTorque) {
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

    const MetaSense::CANBus::Stats& canStatsForStart = MetaSense::CANBus::stats();
    const bool canTelemetryReadyForStart = evaluateCanStartReadiness(now, canStatsForStart);
    
    // INIT state completion criterion: Use the actual inverter status bit from 0x1DA frame
    // This is more direct and concrete than synthetic CAN readiness flags.
    // inv_status_bit = 1 means inverter is ready; this is the real state from hardware.
    const LeafInvFeedback& leafFb = MetaSense::CANBus::feedback();
    const bool inverterStatusFromFrame = (leafFb.inv_status_bit == 1);
    
    const bool canActivityReady = inverterStatusFromFrame || canTelemetryReadyForStart;
    const bool relayInverterStatusReady = canActivityReady || tele.vcuReady;
    const bool relayInverterReady = canActivityReady || tele.vcuReady;
    const bool relayInverterFault = tele.leaf_invFault ||
                                    tele.leaf_invLimp;
    const bool sensorsReadyForStart = isfinite(tele.rpm) &&
                                      isfinite(tele.loadKg) &&
                                      isfinite(tele.lambdaValue);

    const bool telemetryConnectedForStart = MetaSense::WebSocketServer::socket().count() > 0U;
    MetaSense::HardwareOutputStateMachine::update(
        engineThrottlePercent,
        tele.rpmTarget,
        tele.rpm,
        primaryBrakeSignedPercent,
        tele.vcuHvVoltage,
        relayInverterStatusReady,
        relayInverterReady,
        relayInverterFault,
        telemetryConnectedForStart,
        canTelemetryReadyForStart,
        sensorsReadyForStart);

    const bool ssrActiveForLeafTx = MetaSense::HardwareOutputStateMachine::isSsrActive();
    const bool prechargeActiveForLeafTx = MetaSense::HardwareOutputStateMachine::isPrechargeActive();
    const bool prechargeSucceededForLeafTx = MetaSense::HardwareOutputStateMachine::isPrechargeSucceeded();
    const LeafInvFeedback& leafFbDiag = MetaSense::CANBus::feedback();
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
    struct CorrAcc {
        uint32_t n;
        double sx;
        double sy;
        double sxx;
        double syy;
        double sxy;
    };
    auto corrUpdate = [](CorrAcc& c, double x, double y) {
        c.n += 1U;
        c.sx += x;
        c.sy += y;
        c.sxx += x * x;
        c.syy += y * y;
        c.sxy += x * y;
    };
    auto corrValue = [](const CorrAcc& c) -> float {
        if (c.n < 8U) {
            return 0.0f;
        }
        const double n = static_cast<double>(c.n);
        const double num = (n * c.sxy) - (c.sx * c.sy);
        const double denX = (n * c.sxx) - (c.sx * c.sx);
        const double denY = (n * c.syy) - (c.sy * c.sy);
        if (denX <= 1e-9 || denY <= 1e-9) {
            return 0.0f;
        }
        const double den = sqrt(denX * denY);
        if (den <= 1e-9) {
            return 0.0f;
        }
        return static_cast<float>(num / den);
    };
    auto linSlope = [](const CorrAcc& c) -> float {
        if (c.n < 8U) {
            return 0.0f;
        }
        const double n = static_cast<double>(c.n);
        const double denX = (n * c.sxx) - (c.sx * c.sx);
        if (fabs(denX) <= 1e-9) {
            return 0.0f;
        }
        const double num = (n * c.sxy) - (c.sx * c.sy);
        return static_cast<float>(num / denX);
    };
    auto linIntercept = [](const CorrAcc& c, float slope) -> float {
        if (c.n == 0U) {
            return 0.0f;
        }
        return static_cast<float>((c.sy - static_cast<double>(slope) * c.sx) /
                                  static_cast<double>(c.n));
    };

    static CorrAcc s_corr120Be01Torque = {0U, 0.0, 0.0, 0.0, 0.0, 0.0};
    static CorrAcc s_corr120Le01Torque = {0U, 0.0, 0.0, 0.0, 0.0, 0.0};
    static uint32_t s_corrLast120Frames = 0U;
    static bool s_corrTorqueEmaInit = false;
    static float s_corrTorqueEmaNm = 0.0f;
    static uint32_t s_corrFitAcceptedN = 0U;
    static uint32_t s_corrFitZeroHoldSkipN = 0U;
    static bool s_corrFitFrozenNow = false;
    static uint32_t s_corrLast1daFrames = 0U;
    static uint32_t s_corrLast1d4SniffFrames = 0U;
    static uint32_t s_tqCmpAllN = 0U;
    static double s_tqCmpAllAbsErrSum = 0.0;
    static double s_tqCmpAllErrSqSum = 0.0;
    static uint32_t s_tqCmpLowN = 0U;
    static double s_tqCmpLowAbsErrSum = 0.0;
    static double s_tqCmpLowErrSqSum = 0.0;
    static uint32_t s_tqCmpAllClipN = 0U;
    static double s_tqCmpAllClipAbsErrSum = 0.0;
    static double s_tqCmpAllClipErrSqSum = 0.0;
    static uint32_t s_tqCmpLowClipN = 0U;
    static double s_tqCmpLowClipAbsErrSum = 0.0;
    static double s_tqCmpLowClipErrSqSum = 0.0;
    static uint32_t s_tqCmpStateN[3] = {0U, 0U, 0U};
    static double s_tqCmpStateAbsErrSum[3] = {0.0, 0.0, 0.0};
    static double s_tqCmpStateErrSqSum[3] = {0.0, 0.0, 0.0};
    static uint32_t s_tqCmpStateClipN[3] = {0U, 0U, 0U};
    static double s_tqCmpStateClipAbsErrSum[3] = {0.0, 0.0, 0.0};
    static double s_tqCmpStateClipErrSqSum[3] = {0.0, 0.0, 0.0};
    static bool s_cmdZeroBiasInit = false;
    static float s_cmdZeroBiasNm = 0.0f;
    static uint32_t s_cmdZeroBiasN = 0U;

    const MetaSense::CANBus::Stats& canStatsDiag = MetaSense::CANBus::stats();

    if (canStatsDiag.rx1daFrames != s_corrLast1daFrames) {
        const uint32_t crc1daCandidateMask = computeLeaf1daCrcCandidateMask(leafFbDiag.id1da_raw, 8U);
        ++leaf1daCrcSamples;
        for (uint8_t bit = 0U; bit < kLeaf1daCrcCandidateCount; ++bit) {
            if ((crc1daCandidateMask & static_cast<uint32_t>(1U << bit)) != 0U) {
                ++leaf1daCrcCandidateMatches[bit];
            }
        }

        uint8_t crc1daResidues[kLeaf1daResidueCandidateCount] = {0U};
        computeLeaf1daResidues(leafFbDiag.id1da_raw, 8U, crc1daResidues);
        const uint8_t crc1daClock = static_cast<uint8_t>(leafFbDiag.id1da_raw[6] & 0x03U);
        ++leaf1daResidueSamples;
        ++leaf1daClockSamples[crc1daClock];
        for (uint8_t candidate = 0U; candidate < kLeaf1daResidueCandidateCount; ++candidate) {
            ++leaf1daResidueCounts[candidate][crc1daResidues[candidate]];
            ++leaf1daResidueByClock[candidate][crc1daClock][crc1daResidues[candidate]];
        }

        const uint8_t crc1daRx = leafFbDiag.id1da_raw[7];
        const uint8_t crc1daIdLo = MetaSense::LeafCRC::computeBaseIdLo(0xDAU, leafFbDiag.id1da_raw);
        const uint8_t crc1daIdLoResidue = static_cast<uint8_t>(crc1daRx ^ crc1daIdLo);
        const uint32_t idLoResidueCount = ++leaf1daIdLoResidueByClock[crc1daClock][crc1daIdLoResidue];
        if (idLoResidueCount > leaf1daIdLoTopResidueCountByClock[crc1daClock]) {
            leaf1daIdLoTopResidueCountByClock[crc1daClock] = idLoResidueCount;
            leaf1daIdLoTopResidueByClock[crc1daClock] = crc1daIdLoResidue;
        }
        const uint8_t crc1daClockCorrected = computeApprovedLeafFrameCrc(0xDAU, leafFbDiag.id1da_raw);
        if (crc1daClockCorrected == crc1daRx) {
            ++leaf1daIdLoClockCorrectedMatches;
        }

        const uint32_t autosar1daMask = computeLeaf1daAutosarCandidateMask(leafFbDiag.id1da_raw, 8U);
        ++leaf1daAutosarSamples;
        for (uint8_t bit = 0U; bit < kLeaf1daAutosarCandidateCount; ++bit) {
            if ((autosar1daMask & static_cast<uint32_t>(1U << bit)) != 0U) {
                ++leaf1daAutosarCandidateMatches[bit];
            }
        }

        s_corrLast1daFrames = canStatsDiag.rx1daFrames;
    }

    #if !METASENSE_LEAF_1D4_RAW_SNIFF_ONLY
    if (canStatsDiag.rx1d4SniffFrames != s_corrLast1d4SniffFrames && canStatsDiag.last1d4SniffLen >= 8U) {
        const uint8_t crc1d4Clock = static_cast<uint8_t>(extractIntelUnsigned(canStatsDiag.last1d4SniffData,
                                                                              7U,
                                                                              38U,
                                                                              2U) & 0x03U);
        const uint8_t crc1d4Rx = canStatsDiag.last1d4SniffData[7];
        const uint8_t crc1d4IdLo = MetaSense::LeafCRC::computeBaseIdLo(0xD4U, canStatsDiag.last1d4SniffData);
        const uint8_t crc1d4IdLoResidue = static_cast<uint8_t>(crc1d4Rx ^ crc1d4IdLo);
        const uint32_t idLoResidueCount = ++leaf1d4IdLoResidueByClock[crc1d4Clock][crc1d4IdLoResidue];
        if (idLoResidueCount > leaf1d4IdLoTopResidueCountByClock[crc1d4Clock]) {
            leaf1d4IdLoTopResidueCountByClock[crc1d4Clock] = idLoResidueCount;
            leaf1d4IdLoTopResidueByClock[crc1d4Clock] = crc1d4IdLoResidue;
        }
        const uint8_t crc1d4ClockCorrected = static_cast<uint8_t>(crc1d4IdLo ^ leaf1d4IdLoTopResidueByClock[crc1d4Clock]);
        if (crc1d4ClockCorrected == crc1d4Rx) {
            ++leaf1d4IdLoClockCorrectedMatches;
        }

        ++leaf1d4CrcSamples;

        s_corrLast1d4SniffFrames = canStatsDiag.rx1d4SniffFrames;
    }
    #endif

    if (s_leafPreStatusLastMs == 0U || (now - s_leafPreStatusLastMs) >= CAN_PRE_DIAG_LOG_PERIOD_MS) {
        const unsigned long feedbackAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, lastCanLeafAnyUpdate));
        const unsigned long id1daAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.rpm_update_ms));
        const unsigned long id1d4CmdAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last1d4CmdMs));
        const unsigned long id1dbAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.torque_update_ms));
        const unsigned long id1dcAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.temps_update_ms));
        const unsigned long raw11aAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last11aMs));
        const unsigned long raw120AgeMs = static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last120Ms));
        const unsigned long unkAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.lastUnknownMs));
        const unsigned long legacy55xCompat120Count = static_cast<unsigned long>(canStatsDiag.rx120Frames);
        const unsigned long legacy55xCompat120AgeMs = raw120AgeMs;
        const Leaf120CommandDecode id120Decoded = decodeLeaf120Command(canStatsDiag.last120Data,
                                        canStatsDiag.last120Len);
        const Leaf1d4CommandDecode id1d4Cmd = decodeLeaf1d4Command(canStatsDiag.last1d4CmdData,
                                                                    canStatsDiag.last1d4CmdLen);
    #if METASENSE_LEAF_1D4_SNIFF_RX_ENABLED
        const Leaf1d4CommandDecode id1d4Sniff = decodeLeaf1d4Command(canStatsDiag.last1d4SniffData,
                                                                      canStatsDiag.last1d4SniffLen);
        if (id1d4Sniff.valid) {
            Serial.printf("[VCM-1D4-SNIFF] n=%lu age=%lu tqReq=%.2fNm(raw=%d) hv=%u rplus=%u clk=%u crc=0x%02X data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                          static_cast<unsigned long>(canStatsDiag.rx1d4SniffFrames),
                          static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last1d4SniffMs)),
                          id1d4Sniff.motorAmpTorqueNm,
                          static_cast<int>(id1d4Sniff.motorAmpTorqueRaw),
                          id1d4Sniff.hvSupplyStatus ? 1U : 0U,
                          id1d4Sniff.relayPlusStatus ? 1U : 0U,
                          static_cast<unsigned>(id1d4Sniff.hcmClock),
                          static_cast<unsigned>(id1d4Sniff.crc1d4),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffData[0]),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffData[1]),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffData[2]),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffData[3]),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffData[4]),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffData[5]),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffData[6]),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffData[7]));
            Serial0.printf("[VCM-1D4-SNIFF] n=%lu age=%lu tqReq=%.2fNm(raw=%d) hv=%u rplus=%u clk=%u crc=0x%02X data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                           static_cast<unsigned long>(canStatsDiag.rx1d4SniffFrames),
                           static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last1d4SniffMs)),
                           id1d4Sniff.motorAmpTorqueNm,
                           static_cast<int>(id1d4Sniff.motorAmpTorqueRaw),
                           id1d4Sniff.hvSupplyStatus ? 1U : 0U,
                           id1d4Sniff.relayPlusStatus ? 1U : 0U,
                           static_cast<unsigned>(id1d4Sniff.hcmClock),
                           static_cast<unsigned>(id1d4Sniff.crc1d4),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffData[0]),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffData[1]),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffData[2]),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffData[3]),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffData[4]),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffData[5]),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffData[6]),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffData[7]));
        } else {
            Serial.printf("[VCM-1D4-SNIFF] n=%lu age=%lu len=%u (waiting for full 8-byte frame)\n",
                          static_cast<unsigned long>(canStatsDiag.rx1d4SniffFrames),
                          static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last1d4SniffMs)),
                          static_cast<unsigned>(canStatsDiag.last1d4SniffLen));
            Serial0.printf("[VCM-1D4-SNIFF] n=%lu age=%lu len=%u (waiting for full 8-byte frame)\n",
                           static_cast<unsigned long>(canStatsDiag.rx1d4SniffFrames),
                           static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last1d4SniffMs)),
                           static_cast<unsigned>(canStatsDiag.last1d4SniffLen));
        }
#endif
        const int16_t id120Sbe01ForCmd = id120Decoded.torqueDemandSignedBe;
        const float beCorrNow = corrValue(s_corr120Be01Torque);
        const bool useFitModelNow = (s_corr120Be01Torque.n >= METASENSE_LEAF_120_CMD_MIN_FIT_SAMPLES);
        const float beSlopeNow = useFitModelNow
                     ? linSlope(s_corr120Be01Torque)
                     : METASENSE_LEAF_120_CMD_BASE_SLOPE;
        const float beOffsetNow = useFitModelNow
                      ? linIntercept(s_corr120Be01Torque, beSlopeNow)
                      : METASENSE_LEAF_120_CMD_BASE_OFFSET_NM;
        const float id120CmdEstNmNow = beOffsetNow + (beSlopeNow * static_cast<float>(id120Sbe01ForCmd));
        const bool zeroCmdSample = (id120Sbe01ForCmd == 0) &&
                                   (fabsf(leafFbDiag.rpm) <= 100.0f) &&
                                   (fabsf(leafFbDiag.torque_nm) <= 0.10f);
        if (zeroCmdSample) {
            if (!s_cmdZeroBiasInit) {
                s_cmdZeroBiasNm = id120CmdEstNmNow;
                s_cmdZeroBiasInit = true;
            } else {
                const float biasDeltaNm = fabsf(id120CmdEstNmNow - s_cmdZeroBiasNm);
                if (s_cmdZeroBiasN < 5U || biasDeltaNm > 0.50f) {
                    // Fast-lock bias when entering a stable zero-command hold.
                    s_cmdZeroBiasNm = id120CmdEstNmNow;
                } else {
                    const float kZeroBiasAlpha = 0.20f;
                    s_cmdZeroBiasNm += kZeroBiasAlpha * (id120CmdEstNmNow - s_cmdZeroBiasNm);
                }
            }
            s_cmdZeroBiasN += 1U;
        }
        const bool applyZeroBiasNow = s_corrFitFrozenNow || zeroCmdSample;
        const float id120CmdAdjNmNow = applyZeroBiasNow
                         ? (id120CmdEstNmNow - s_cmdZeroBiasNm)
                         : id120CmdEstNmNow;
        const float tqEffNmNow = leafFbDiag.torque_nm;
        const float tqErrNmNow = tqEffNmNow - id120CmdEstNmNow;
        const float tqAbsErrNmNow = fabsf(tqErrNmNow);
        const float tqErrAdjNmNow = tqEffNmNow - id120CmdAdjNmNow;
        const float tqAbsErrAdjNmNow = fabsf(tqErrAdjNmNow);
        const float tqClipAbsNm = METASENSE_LEAF_120_NOISE_OUTLIER_ABS_NM;
        const bool tqErrAdjClipped = tqAbsErrAdjNmNow <= tqClipAbsNm;
        constexpr uint8_t kTqCmpStateZero = 0U;
        constexpr uint8_t kTqCmpStateMotor = 1U;
        constexpr uint8_t kTqCmpStateRegen = 2U;
        uint8_t tqCmpState = kTqCmpStateZero;
        if (id120CmdAdjNmNow > METASENSE_LEAF_120_NOISE_DB_NM) {
            tqCmpState = kTqCmpStateMotor;
        } else if (id120CmdAdjNmNow < -METASENSE_LEAF_120_NOISE_DB_NM) {
            tqCmpState = kTqCmpStateRegen;
        }

        // Track agreement quality between estimated 0x120 command and effective torque.
        s_tqCmpAllN += 1U;
        s_tqCmpAllAbsErrSum += static_cast<double>(tqAbsErrAdjNmNow);
        s_tqCmpAllErrSqSum += static_cast<double>(tqErrAdjNmNow) * static_cast<double>(tqErrAdjNmNow);
        s_tqCmpStateN[tqCmpState] += 1U;
        s_tqCmpStateAbsErrSum[tqCmpState] += static_cast<double>(tqAbsErrAdjNmNow);
        s_tqCmpStateErrSqSum[tqCmpState] += static_cast<double>(tqErrAdjNmNow) * static_cast<double>(tqErrAdjNmNow);
        if (tqErrAdjClipped) {
            s_tqCmpAllClipN += 1U;
            s_tqCmpAllClipAbsErrSum += static_cast<double>(tqAbsErrAdjNmNow);
            s_tqCmpAllClipErrSqSum += static_cast<double>(tqErrAdjNmNow) * static_cast<double>(tqErrAdjNmNow);
            s_tqCmpStateClipN[tqCmpState] += 1U;
            s_tqCmpStateClipAbsErrSum[tqCmpState] += static_cast<double>(tqAbsErrAdjNmNow);
            s_tqCmpStateClipErrSqSum[tqCmpState] += static_cast<double>(tqErrAdjNmNow) * static_cast<double>(tqErrAdjNmNow);
        }

        const bool lowTorqueWindow = (fabsf(tqEffNmNow) <= 5.0f) && (fabsf(id120CmdAdjNmNow) <= 5.0f);
        if (lowTorqueWindow) {
            s_tqCmpLowN += 1U;
            s_tqCmpLowAbsErrSum += static_cast<double>(tqAbsErrAdjNmNow);
            s_tqCmpLowErrSqSum += static_cast<double>(tqErrAdjNmNow) * static_cast<double>(tqErrAdjNmNow);
            if (tqErrAdjClipped) {
                s_tqCmpLowClipN += 1U;
                s_tqCmpLowClipAbsErrSum += static_cast<double>(tqAbsErrAdjNmNow);
                s_tqCmpLowClipErrSqSum += static_cast<double>(tqErrAdjNmNow) * static_cast<double>(tqErrAdjNmNow);
            }
        }

        const float tqMaeAllNm = (s_tqCmpAllN > 0U)
                                     ? static_cast<float>(s_tqCmpAllAbsErrSum / static_cast<double>(s_tqCmpAllN))
                                     : 0.0f;
        const float tqRmseAllNm = (s_tqCmpAllN > 0U)
                                      ? static_cast<float>(sqrt(s_tqCmpAllErrSqSum / static_cast<double>(s_tqCmpAllN)))
                                      : 0.0f;
        const float tqMaeLowNm = (s_tqCmpLowN > 0U)
                                     ? static_cast<float>(s_tqCmpLowAbsErrSum / static_cast<double>(s_tqCmpLowN))
                                     : 0.0f;
        const float tqRmseLowNm = (s_tqCmpLowN > 0U)
                                      ? static_cast<float>(sqrt(s_tqCmpLowErrSqSum / static_cast<double>(s_tqCmpLowN)))
                                      : 0.0f;
        const float tqMaeAllClipNm = (s_tqCmpAllClipN > 0U)
                         ? static_cast<float>(s_tqCmpAllClipAbsErrSum / static_cast<double>(s_tqCmpAllClipN))
                         : 0.0f;
        const float tqRmseAllClipNm = (s_tqCmpAllClipN > 0U)
                          ? static_cast<float>(sqrt(s_tqCmpAllClipErrSqSum / static_cast<double>(s_tqCmpAllClipN)))
                          : 0.0f;
        const float tqMaeLowClipNm = (s_tqCmpLowClipN > 0U)
                         ? static_cast<float>(s_tqCmpLowClipAbsErrSum / static_cast<double>(s_tqCmpLowClipN))
                         : 0.0f;
        const float tqRmseLowClipNm = (s_tqCmpLowClipN > 0U)
                          ? static_cast<float>(sqrt(s_tqCmpLowClipErrSqSum / static_cast<double>(s_tqCmpLowClipN)))
                          : 0.0f;
        const float tqMaeStateZeroClipNm = (s_tqCmpStateClipN[kTqCmpStateZero] > 0U)
                 ? static_cast<float>(s_tqCmpStateClipAbsErrSum[kTqCmpStateZero] / static_cast<double>(s_tqCmpStateClipN[kTqCmpStateZero]))
                 : 0.0f;
        const float tqRmseStateZeroClipNm = (s_tqCmpStateClipN[kTqCmpStateZero] > 0U)
                  ? static_cast<float>(sqrt(s_tqCmpStateClipErrSqSum[kTqCmpStateZero] / static_cast<double>(s_tqCmpStateClipN[kTqCmpStateZero])))
                  : 0.0f;
        const float tqMaeStateMotorClipNm = (s_tqCmpStateClipN[kTqCmpStateMotor] > 0U)
                  ? static_cast<float>(s_tqCmpStateClipAbsErrSum[kTqCmpStateMotor] / static_cast<double>(s_tqCmpStateClipN[kTqCmpStateMotor]))
                  : 0.0f;
        const float tqRmseStateMotorClipNm = (s_tqCmpStateClipN[kTqCmpStateMotor] > 0U)
                   ? static_cast<float>(sqrt(s_tqCmpStateClipErrSqSum[kTqCmpStateMotor] / static_cast<double>(s_tqCmpStateClipN[kTqCmpStateMotor])))
                   : 0.0f;
        const float tqMaeStateRegenClipNm = (s_tqCmpStateClipN[kTqCmpStateRegen] > 0U)
                  ? static_cast<float>(s_tqCmpStateClipAbsErrSum[kTqCmpStateRegen] / static_cast<double>(s_tqCmpStateClipN[kTqCmpStateRegen]))
                  : 0.0f;
        const float tqRmseStateRegenClipNm = (s_tqCmpStateClipN[kTqCmpStateRegen] > 0U)
                   ? static_cast<float>(sqrt(s_tqCmpStateClipErrSqSum[kTqCmpStateRegen] / static_cast<double>(s_tqCmpStateClipN[kTqCmpStateRegen])))
                   : 0.0f;
        if (lastCanAltDiagMs == 0U || (now - lastCanAltDiagMs) >= CAN_ALT_DIAG_LOG_PERIOD_MS) {
            char posTop[64];
            char negTop[64];
            char zeroTop[64];
            char brakeTop[64];
            formatLeaf120TopValues(leaf120B2PosCounts, posTop, sizeof(posTop));
            formatLeaf120TopValues(leaf120B2NegCounts, negTop, sizeof(negTop));
            formatLeaf120TopValues(leaf120B2ZeroCounts, zeroTop, sizeof(zeroTop));
            formatLeaf120TopValues(leaf120B2BrakeCounts, brakeTop, sizeof(brakeTop));

            if (canStatsDiag.rx120Frames > 0U) {
                Serial.printf("[VCM-120-DISC] frames=%lu dB2 pos(n=%lu top=%s) neg(n=%lu top=%s) zero(n=%lu top=%s) brake(n=%lu top=%s)\n",
                              static_cast<unsigned long>(canStatsDiag.rx120Frames),
                              static_cast<unsigned long>(leaf120B2PosSamples),
                              posTop,
                              static_cast<unsigned long>(leaf120B2NegSamples),
                              negTop,
                              static_cast<unsigned long>(leaf120B2ZeroSamples),
                              zeroTop,
                              static_cast<unsigned long>(leaf120B2BrakeSamples),
                              brakeTop);
                Serial0.printf("[VCM-120-DISC] frames=%lu dB2 pos(n=%lu top=%s) neg(n=%lu top=%s) zero(n=%lu top=%s) brake(n=%lu top=%s)\n",
                               static_cast<unsigned long>(canStatsDiag.rx120Frames),
                               static_cast<unsigned long>(leaf120B2PosSamples),
                               posTop,
                               static_cast<unsigned long>(leaf120B2NegSamples),
                               negTop,
                               static_cast<unsigned long>(leaf120B2ZeroSamples),
                               zeroTop,
                               static_cast<unsigned long>(leaf120B2BrakeSamples),
                               brakeTop);
            }
            const float crc120StateFixPctAll = (leaf120CrcSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatches) / static_cast<float>(leaf120CrcSamples))
                : 0.0f;
            const float crc120Fixed8PctAll = (leaf120CrcSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8Matches) / static_cast<float>(leaf120CrcSamples))
                : 0.0f;
            if (leaf120CrcSamples > 0U) {
                Serial.printf("[VCM-120-CRC-OK] stateFix=%lu/%lu(%.1f%%) fixed8=%lu/%lu(%.1f%%)\n",
                              static_cast<unsigned long>(leaf120Ref00StateCorrectedMatches),
                              static_cast<unsigned long>(leaf120CrcSamples),
                              crc120StateFixPctAll,
                              static_cast<unsigned long>(leaf120Ref00Fixed8Matches),
                              static_cast<unsigned long>(leaf120CrcSamples),
                              crc120Fixed8PctAll);
                Serial0.printf("[VCM-120-CRC-OK] stateFix=%lu/%lu(%.1f%%) fixed8=%lu/%lu(%.1f%%)\n",
                               static_cast<unsigned long>(leaf120Ref00StateCorrectedMatches),
                               static_cast<unsigned long>(leaf120CrcSamples),
                               crc120StateFixPctAll,
                               static_cast<unsigned long>(leaf120Ref00Fixed8Matches),
                               static_cast<unsigned long>(leaf120CrcSamples),
                               crc120Fixed8PctAll);
            }
            const float txExpExactPct = (leaf120TxExpSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120TxExpExactCrcMatches) / static_cast<float>(leaf120TxExpSamples))
                : 0.0f;
            const float txExpHiCtrCrcPct = (leaf120TxExpSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120TxExpHiCtrCrcMatches) / static_cast<float>(leaf120TxExpSamples))
                : 0.0f;
            const float txExpLoCtrCrcPct = (leaf120TxExpSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120TxExpLoCtrCrcMatches) / static_cast<float>(leaf120TxExpSamples))
                : 0.0f;
            const float txExpHiCtrB2Pct = (leaf120TxExpSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120TxExpHiCtrB2Matches) / static_cast<float>(leaf120TxExpSamples))
                : 0.0f;
            const float txExpLoCtrB2Pct = (leaf120TxExpSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120TxExpLoCtrB2Matches) / static_cast<float>(leaf120TxExpSamples))
                : 0.0f;
            if (leaf120TxExpSamples > 0U) {
                Serial.printf("[VCM-120-TX-EXP] n=%lu exact_crc=%.1f%% hiCtr(crc=%.1f%%,b2=%.1f%%) loCtr(crc=%.1f%%,b2=%.1f%%)\n",
                              static_cast<unsigned long>(leaf120TxExpSamples),
                              txExpExactPct,
                              txExpHiCtrCrcPct,
                              txExpHiCtrB2Pct,
                              txExpLoCtrCrcPct,
                              txExpLoCtrB2Pct);
                Serial0.printf("[VCM-120-TX-EXP] n=%lu exact_crc=%.1f%% hiCtr(crc=%.1f%%,b2=%.1f%%) loCtr(crc=%.1f%%,b2=%.1f%%)\n",
                               static_cast<unsigned long>(leaf120TxExpSamples),
                               txExpExactPct,
                               txExpHiCtrCrcPct,
                               txExpHiCtrB2Pct,
                               txExpLoCtrCrcPct,
                               txExpLoCtrB2Pct);
            }
        #if !METASENSE_LEAF_1D4_RAW_SNIFF_ONLY
            const float crc1daClockFixPct = (leaf1daCrcSamples > 0U)
                ? (100.0f * static_cast<float>(leaf1daIdLoClockCorrectedMatches) / static_cast<float>(leaf1daCrcSamples))
                : 0.0f;
            Serial.printf("[VCM-1DA-CRC-OK] clockFix=%lu/%lu(%.1f%%)\n",
                          static_cast<unsigned long>(leaf1daIdLoClockCorrectedMatches),
                          static_cast<unsigned long>(leaf1daCrcSamples),
                          crc1daClockFixPct);
            Serial0.printf("[VCM-1DA-CRC-OK] clockFix=%lu/%lu(%.1f%%)\n",
                           static_cast<unsigned long>(leaf1daIdLoClockCorrectedMatches),
                           static_cast<unsigned long>(leaf1daCrcSamples),
                           crc1daClockFixPct);
            const float crc1d4ClockFixPct = (leaf1d4CrcSamples > 0U)
                ? (100.0f * static_cast<float>(leaf1d4IdLoClockCorrectedMatches) / static_cast<float>(leaf1d4CrcSamples))
                : 0.0f;
            Serial.printf("[VCM-1D4-CRC-STATS] frames=%lu match(c1D_id1d4lo=%lu)\n",
                          static_cast<unsigned long>(leaf1d4CrcSamples),
                          static_cast<unsigned long>(leaf1d4CrcCandidateMatches[15]));
            Serial0.printf("[VCM-1D4-CRC-STATS] frames=%lu match(c1D_id1d4lo=%lu)\n",
                           static_cast<unsigned long>(leaf1d4CrcSamples),
                           static_cast<unsigned long>(leaf1d4CrcCandidateMatches[15]));
            Serial.printf("[VCM-1D4-CRC-OK] clockFix=%lu/%lu(%.1f%%)\n",
                          static_cast<unsigned long>(leaf1d4IdLoClockCorrectedMatches),
                          static_cast<unsigned long>(leaf1d4CrcSamples),
                          crc1d4ClockFixPct);
            Serial0.printf("[VCM-1D4-CRC-OK] clockFix=%lu/%lu(%.1f%%)\n",
                           static_cast<unsigned long>(leaf1d4IdLoClockCorrectedMatches),
                           static_cast<unsigned long>(leaf1d4CrcSamples),
                           crc1d4ClockFixPct);
            Serial.printf("[VCM-1D4-CRC4-STATS] idlo_clock_fix(match=%lu/%lu c0=0x%02X:%lu c1=0x%02X:%lu c2=0x%02X:%lu c3=0x%02X:%lu)\n",
                          static_cast<unsigned long>(leaf1d4IdLoClockCorrectedMatches),
                          static_cast<unsigned long>(leaf1d4CrcSamples),
                          static_cast<unsigned>(leaf1d4IdLoTopResidueByClock[0]),
                          static_cast<unsigned long>(leaf1d4IdLoTopResidueCountByClock[0]),
                          static_cast<unsigned>(leaf1d4IdLoTopResidueByClock[1]),
                          static_cast<unsigned long>(leaf1d4IdLoTopResidueCountByClock[1]),
                          static_cast<unsigned>(leaf1d4IdLoTopResidueByClock[2]),
                          static_cast<unsigned long>(leaf1d4IdLoTopResidueCountByClock[2]),
                          static_cast<unsigned>(leaf1d4IdLoTopResidueByClock[3]),
                          static_cast<unsigned long>(leaf1d4IdLoTopResidueCountByClock[3]));
            Serial0.printf("[VCM-1D4-CRC4-STATS] idlo_clock_fix(match=%lu/%lu c0=0x%02X:%lu c1=0x%02X:%lu c2=0x%02X:%lu c3=0x%02X:%lu)\n",
                           static_cast<unsigned long>(leaf1d4IdLoClockCorrectedMatches),
                           static_cast<unsigned long>(leaf1d4CrcSamples),
                           static_cast<unsigned>(leaf1d4IdLoTopResidueByClock[0]),
                           static_cast<unsigned long>(leaf1d4IdLoTopResidueCountByClock[0]),
                           static_cast<unsigned>(leaf1d4IdLoTopResidueByClock[1]),
                           static_cast<unsigned long>(leaf1d4IdLoTopResidueCountByClock[1]),
                           static_cast<unsigned>(leaf1d4IdLoTopResidueByClock[2]),
                           static_cast<unsigned long>(leaf1d4IdLoTopResidueCountByClock[2]),
                           static_cast<unsigned>(leaf1d4IdLoTopResidueByClock[3]),
                           static_cast<unsigned long>(leaf1d4IdLoTopResidueCountByClock[3]));
        #endif
            lastCanAltDiagMs = now;
            lastCanAlt120FramesLogged = canStatsDiag.rx120Frames;
        }
        const uint16_t id120Le01 = static_cast<uint16_t>(canStatsDiag.last120Data[0]) |
                                   (static_cast<uint16_t>(canStatsDiag.last120Data[1]) << 8);
        const uint16_t id120Le23 = static_cast<uint16_t>(canStatsDiag.last120Data[2]) |
                                   (static_cast<uint16_t>(canStatsDiag.last120Data[3]) << 8);
        const uint16_t id120Be01 = id120Decoded.torqueDemandRawBe;
        const uint16_t id120Be23 = (static_cast<uint16_t>(canStatsDiag.last120Data[2]) << 8) |
                                   static_cast<uint16_t>(canStatsDiag.last120Data[3]);
        const uint16_t id11aTail = (static_cast<uint16_t>(canStatsDiag.last11aData[6]) << 8) |
                                  static_cast<uint16_t>(canStatsDiag.last11aData[7]);
        const int16_t id120Sbe01 = static_cast<int16_t>(id120Be01);
        const int16_t id120Sle01 = static_cast<int16_t>(id120Le01);
        const uint8_t id120Ctr = id120Decoded.unknown120_2;
        const uint8_t id120Crc = id120Decoded.crc120;
        const uint8_t id120Xor = static_cast<uint8_t>(canStatsDiag.last120Data[0] ^
                                  canStatsDiag.last120Data[1] ^
                                  canStatsDiag.last120Data[2]);

    #if METASENSE_LEAF_MONITOR_SIMPLE_LOGS
        static uint32_t s_simpleLogDecimator = 0U;
        const char* tqModeStr = (tqCmpState == kTqCmpStateMotor) ? "MOTOR" :
                                ((tqCmpState == kTqCmpStateRegen) ? "REGEN" : "ZERO");
        const bool emitSimpleLine = (METASENSE_LEAF_MONITOR_SIMPLE_DECIMATE <= 1U) ||
                                    ((++s_simpleLogDecimator % METASENSE_LEAF_MONITOR_SIMPLE_DECIMATE) == 0U);
        if (emitSimpleLine) {
            Serial.printf("[VCM-SIMPLE] mode=%s fb1da=%.2fNm ageMs(tq=%lu)\n",
                          tqModeStr,
                          tqEffNmNow,
                          id1dbAgeMs);
            Serial0.printf("[VCM-SIMPLE] mode=%s fb1da=%.2fNm ageMs(tq=%lu)\n",
                           tqModeStr,
                           tqEffNmNow,
                           id1dbAgeMs);
        }
    #else
        Serial.printf("[VCM-MAP] id120(cmd_raw=%d cmd120_est_nm=%.2f cmd120_adj_nm=%.2f fb1da_nm=%.2f delta_nm=%.2f unknown_120_2=%u crc_120=0x%02X xor=0x%02X model=%s corr=%.3f n=%lu compat_le01_s=%d compat_le01_u=%u compat_be23=%u compat_le23=%u compat11a_tail=0x%04X)\n",
                  static_cast<int>(id120Sbe01),
                  id120CmdEstNmNow,
                  id120CmdAdjNmNow,
                  tqEffNmNow,
                  tqErrAdjNmNow,
                  static_cast<unsigned>(id120Ctr),
                  static_cast<unsigned>(id120Crc),
                  static_cast<unsigned>(id120Xor),
                  useFitModelNow ? "fit" : "base",
                  beCorrNow,
                  static_cast<unsigned long>(s_corr120Be01Torque.n),
                  static_cast<int>(id120Sle01),
                  static_cast<unsigned>(id120Le01),
                  static_cast<unsigned>(id120Be23),
                  static_cast<unsigned>(id120Le23),
                  static_cast<unsigned>(id11aTail));
        Serial0.printf("[VCM-MAP] id120(cmd_raw=%d cmd120_est_nm=%.2f cmd120_adj_nm=%.2f fb1da_nm=%.2f delta_nm=%.2f unknown_120_2=%u crc_120=0x%02X xor=0x%02X model=%s corr=%.3f n=%lu compat_le01_s=%d compat_le01_u=%u compat_be23=%u compat_le23=%u compat11a_tail=0x%04X)\n",
                   static_cast<int>(id120Sbe01),
                   id120CmdEstNmNow,
                   id120CmdAdjNmNow,
                   tqEffNmNow,
                   tqErrAdjNmNow,
                   static_cast<unsigned>(id120Ctr),
                   static_cast<unsigned>(id120Crc),
                   static_cast<unsigned>(id120Xor),
                   useFitModelNow ? "fit" : "base",
                   beCorrNow,
                   static_cast<unsigned long>(s_corr120Be01Torque.n),
                   static_cast<int>(id120Sle01),
                   static_cast<unsigned>(id120Le01),
                   static_cast<unsigned>(id120Be23),
                   static_cast<unsigned>(id120Le23),
                   static_cast<unsigned>(id11aTail));


                                                                Serial.printf("[VCM-TQ-CMP] fb1da_nm=%.2f cmd120_est_nm=%.2f cmd120_adj_nm=%.2f biasMode=%d delta_est_nm=%.2f delta_adj_nm=%.2f absDelta_est_nm=%.2f absDelta_adj_nm=%.2f clipThr=%.2f inClip=%d maeAll=%.2f rmseAll=%.2f nAll=%lu maeAllClip=%.2f rmseAllClip=%.2f nAllClip=%lu maeLow=%.2f rmseLow=%.2f nLow=%lu maeLowClip=%.2f rmseLowClip=%.2f nLowClip=%lu stateClip(z=%lu %.2f/%.2f,m=%lu %.2f/%.2f,r=%lu %.2f/%.2f) lowWin=%d zeroBiasNm=%.2f nBias=%lu ageMs(tq=%lu,120=%lu)\n",
                              tqEffNmNow,
                              id120CmdEstNmNow,
                              id120CmdAdjNmNow,
                              applyZeroBiasNow ? 1 : 0,
                              tqErrNmNow,
                              tqErrAdjNmNow,
                              tqAbsErrNmNow,
                              tqAbsErrAdjNmNow,
                                                            tqClipAbsNm,
                                                            tqErrAdjClipped ? 1 : 0,
                              tqMaeAllNm,
                              tqRmseAllNm,
                              static_cast<unsigned long>(s_tqCmpAllN),
                                                            tqMaeAllClipNm,
                                                            tqRmseAllClipNm,
                                                            static_cast<unsigned long>(s_tqCmpAllClipN),
                              tqMaeLowNm,
                              tqRmseLowNm,
                              static_cast<unsigned long>(s_tqCmpLowN),
                                                            tqMaeLowClipNm,
                                                            tqRmseLowClipNm,
                                                            static_cast<unsigned long>(s_tqCmpLowClipN),
                                                            static_cast<unsigned long>(s_tqCmpStateClipN[kTqCmpStateZero]),
                                                            tqMaeStateZeroClipNm,
                                                            tqRmseStateZeroClipNm,
                                                            static_cast<unsigned long>(s_tqCmpStateClipN[kTqCmpStateMotor]),
                                                            tqMaeStateMotorClipNm,
                                                            tqRmseStateMotorClipNm,
                                                            static_cast<unsigned long>(s_tqCmpStateClipN[kTqCmpStateRegen]),
                                                            tqMaeStateRegenClipNm,
                                                            tqRmseStateRegenClipNm,
                              lowTorqueWindow ? 1 : 0,
                              s_cmdZeroBiasNm,
                              static_cast<unsigned long>(s_cmdZeroBiasN),
                              id1dbAgeMs,
                              raw120AgeMs);
                                                                Serial0.printf("[VCM-TQ-CMP] fb1da_nm=%.2f cmd120_est_nm=%.2f cmd120_adj_nm=%.2f biasMode=%d delta_est_nm=%.2f delta_adj_nm=%.2f absDelta_est_nm=%.2f absDelta_adj_nm=%.2f clipThr=%.2f inClip=%d maeAll=%.2f rmseAll=%.2f nAll=%lu maeAllClip=%.2f rmseAllClip=%.2f nAllClip=%lu maeLow=%.2f rmseLow=%.2f nLow=%lu maeLowClip=%.2f rmseLowClip=%.2f nLowClip=%lu stateClip(z=%lu %.2f/%.2f,m=%lu %.2f/%.2f,r=%lu %.2f/%.2f) lowWin=%d zeroBiasNm=%.2f nBias=%lu ageMs(tq=%lu,120=%lu)\n",
                               tqEffNmNow,
                               id120CmdEstNmNow,
                               id120CmdAdjNmNow,
                               applyZeroBiasNow ? 1 : 0,
                               tqErrNmNow,
                               tqErrAdjNmNow,
                               tqAbsErrNmNow,
                               tqAbsErrAdjNmNow,
                                                             tqClipAbsNm,
                                                             tqErrAdjClipped ? 1 : 0,
                               tqMaeAllNm,
                               tqRmseAllNm,
                               static_cast<unsigned long>(s_tqCmpAllN),
                                                             tqMaeAllClipNm,
                                                             tqRmseAllClipNm,
                                                             static_cast<unsigned long>(s_tqCmpAllClipN),
                               tqMaeLowNm,
                               tqRmseLowNm,
                               static_cast<unsigned long>(s_tqCmpLowN),
                                                             tqMaeLowClipNm,
                                                             tqRmseLowClipNm,
                                                             static_cast<unsigned long>(s_tqCmpLowClipN),
                                                               static_cast<unsigned long>(s_tqCmpStateClipN[kTqCmpStateZero]),
                                                               tqMaeStateZeroClipNm,
                                                               tqRmseStateZeroClipNm,
                                                               static_cast<unsigned long>(s_tqCmpStateClipN[kTqCmpStateMotor]),
                                                               tqMaeStateMotorClipNm,
                                                               tqRmseStateMotorClipNm,
                                                               static_cast<unsigned long>(s_tqCmpStateClipN[kTqCmpStateRegen]),
                                                               tqMaeStateRegenClipNm,
                                                               tqRmseStateRegenClipNm,
                               lowTorqueWindow ? 1 : 0,
                               s_cmdZeroBiasNm,
                               static_cast<unsigned long>(s_cmdZeroBiasN),
                               id1dbAgeMs,
                               raw120AgeMs);
    #endif
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

                    s_leafRxAwaitPartnerLastMs = now;
                }
            } else if (s_leafRxWarnLastMs == 0U || (now - s_leafRxWarnLastMs) >= CAN_RX_MISSING_LOG_PERIOD_MS) {

                s_leafRxWarnLastMs = now;
            }
        } else {
            s_leafRxWarnLastMs = 0U;
        }

        s_leafRxDiagLastMs = now;
    }

    const bool canBusReadyNow = MetaSense::CANBus::isReady();
    const bool leafTxStartAllowed = canBusReadyNow;
    const bool leafTxChecklistActive = leafTxStartAllowed &&
                                       (kLeafCanTxActive ||
                                        (kLeafCanHandshakeOnFirst1da && s_leafHandshakeSent));

    if (!leafTxChecklistActive) {
        s_leafTxPacerEnabled = false;
        if (!s_leafListenOnlyLogPrinted) {
            Serial.println("[VCM] Passive RX mode active: periodic TX checklist/state machine disabled");
            Serial0.println("[VCM] Passive RX mode active: periodic TX checklist/state machine disabled");
            s_leafListenOnlyLogPrinted = true;
        }

#if (METASENSE_LEAF_120_TX_COMMIT_ENABLED == 0)
        const bool hvOkPreview = tele.vcuReady;
        const bool inverterReadyPreview = tele.leaf_invReady;
        const bool brakePreview = (tele.mode == MetaSense::DynoMode::Brake);
        const bool gearDrivePreview = (tele.rpmTarget > 100.0f) || tele.recording;
        const float uiTorquePreview = getLeafUiTorqueDemandNmInternal();
        const bool manualPreviewActive = fabsf(uiTorquePreview) > 0.001f;
        const float previewTorqueNm = uiTorquePreview;

        // In passive mode, the 1D4 send path remains manual-only and no obsolete trapezoid mode is used.
        if (manualPreviewActive && ((now - lastLeafTxMs) >= CAN_TX_PERIOD_MS)) {
            const bool sentManual = MetaSense::Input::sendLeafTorqueCommand1d4(previewTorqueNm,
                                                                                inverterReadyPreview,
                                                                                hvOkPreview,
                                                                                brakePreview,
                                                                                gearDrivePreview);
            // 0x120 shadow diagnostics removed
            lastLeafTxMs = now;
        }

        // Generate a sampled 0x1D4 frame snapshot for monitor visibility at telemetry cadence.
        if ((now - lastLeaf1d4MonitorSampleMs) >= LEAF_1D4_MONITOR_SAMPLE_PERIOD_MS) {
            const float torquePreview = ((safe && hvOkPreview && inverterReadyPreview) ? torqueCmd : 0.0f);
            (void)MetaSense::Input::sendLeafTorqueCommand1d4(torquePreview,
                                                             inverterReadyPreview,
                                                             hvOkPreview,
                                                             brakePreview,
                                                             gearDrivePreview);
            lastLeaf1d4MonitorSampleMs = now;
        }
#endif

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
                canBusReadyNow &&
                attemptDue &&
                withinWindow &&
                s_leafHandshakeAttemptCount < kLeafHandshakeMaxAttempts) {
                s_leafHandshakeLastAttemptMs = now;
                ++s_leafHandshakeAttemptCount;
                const bool sent = MetaSense::Input::sendLeafTorqueCommand1d4(0.0f,
                                                                             false,
                                                                             false,
                                                                             false,
                                                                             true);
                // 0x120 shadow diagnostics removed
                if (sent) {
                    ++s_leafHandshakeSentCount;
                }
            }

            if (s_leafHandshakeArmed &&
                (s_leafHandshakeSentCount >= kLeafHandshakeTargetSends ||
                 s_leafHandshakeAttemptCount >= kLeafHandshakeMaxAttempts ||
                 !withinWindow)) {
                s_leafHandshakeArmed = false;
                s_leafHandshakeSent = (s_leafHandshakeSentCount > 0U);
            }
        }
    } else if ((now - lastLeafTxMs) >= CAN_TX_PERIOD_MS) {
        if (kLeafCanHandshakeOnFirst1da && !kLeafCanTxActive && !s_leafHandshakePromotedLogPrinted) {
            Serial.println("[VCM] Handshake complete: promoting to continuous 0x120 control mode");
            Serial0.println("[VCM] Handshake complete: promoting to continuous 0x120 control mode");
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
        const bool id1daFreshForTorque = (aliveFb.rpm_update_ms != 0U) &&
                         (elapsedMsSafe(now, aliveFb.rpm_update_ms) <= CAN_RX_TARGET_MAX_AGE_MS);
        const bool id1daReadyTokenObserved = id1daFreshForTorque &&
                                             (aliveFb.id1da_status_bits == 0U);
        const bool id1daStatusClearForTorque = id1daFreshForTorque &&
                               (aliveFb.mg_error_codes == 0U);
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
        const float uiTorqueDemandNm = getLeafUiTorqueDemandNmInternal();

        // No-TVCU startup must not claim ready/HV just because the local controller
        // is alive. The Leaf expects a qualified startup path (partner seen, bus live,
        // safe state, no fault) before the status bits may rise out of the 000/000
        // window; otherwise the inverter latches a 011 style startup fault.
        const bool startupQualified = safe && canBusReady && s_leafCanPartnerSeen && !inverterFaulted;
        // The Leaf uses the all-zero 0x1DA status nibble as a ready token once it
        // has accepted the startup handshake; treat that as a permanent ready signal.
        const bool inverterReadyToken = id1daReadyTokenObserved || tele.leaf_invReady;
        const bool hvOk = startupQualified && (tele.vcuReady || inverterReadyToken);
        const bool inverterReadyBit = startupQualified && inverterReadyToken;
        readyBit = inverterReadyBit;
        hvOkBit = hvOk;
        brakeBit = startupQualified && (tele.mode == MetaSense::DynoMode::Brake);
        gearDriveBit = startupQualified && ((tele.rpmTarget > 100.0f) || tele.recording);
        torqueToSend = (startupQualified && hvOk && inverterReadyBit) ? torqueCmd : 0.0f;

        // Check hardware state: INIT always sends 0 Nm, IDLE/MOTOR can send manual/auto value
        const char* hwState = MetaSense::HardwareOutputStateMachine::stateName();
        const bool isInitState = (hwState != nullptr) && (strcmp(hwState, "INIT") == 0);
        
        // Safety: Auto-reset manual torque when entering INIT state
        if (isInitState && s_lastHardwareState != hwState) {
            setLeafUiTorqueDemandNmInternal(0.0f);
            Serial.println("[SAFETY] Manual torque reset to 0.0 Nm - entering INIT state");
        }
        s_lastHardwareState = hwState;
        
        if (isInitState) {
            // INIT state: force torque demand to 0 Nm during startup (safety critical)
            torqueToSend = 0.0f;
        } else {
            // IDLE/MOTOR states: apply manual/auto torque selection
            const bool manualCommandActive = s_leafCanPartnerSeen && !s_leafSimFeedbackActive &&
                                            (fabsf(uiTorqueDemandNm) > 0.001f);
            const bool torqueCommandUnlocked = safetyOk &&
                                               (s_leafVcmState == LeafVcmBringupState::Ready) &&
                                               id1daFreshForTorque &&
                                               id1daStatusClearForTorque &&
                                               s_leafCanPartnerSeen &&
                                               !s_leafSimFeedbackActive;
            if (manualCommandActive || torqueCommandUnlocked) {
                torqueToSend = uiTorqueDemandNm;
            } else if (fabsf(torqueToSend) > 0.0f) {
                torqueToSend = 0.0f;
            }
        }


        bool skipTxForGapTest = false;


        if (!skipTxForGapTest) {
            s_leafTxPacerTorqueNm = torqueToSend;
            s_leafTxPacerReadyBit = readyBit;
            s_leafTxPacerHvOkBit = hvOkBit;
            s_leafTxPacerBrakeBit = brakeBit;
            s_leafTxPacerGearDriveBit = gearDriveBit;
            s_leafTxPacerEnabled = true;
        } else {
            s_leafTxPacerEnabled = false;
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
                                    (canStats.txFrames != lastCanDiagTxFrames) ||
                                    (canStats.tx1d4Frames != lastCanDiagTx1d4Frames) ||
                                    (canStats.tx11aFrames != lastCanDiagTx11aFrames) ||
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
            uint8_t tx1d4ChangeMask = 0U;
            const uint8_t tx1d4Len = canStats.last1d4TxLen;
            const uint8_t tx1d4CmpLen = (tx1d4Len < lastCanDiag1d4TxLen) ? tx1d4Len : lastCanDiag1d4TxLen;
            for (uint8_t bi = 0U; bi < tx1d4CmpLen; ++bi) {
                if (canStats.last1d4TxData[bi] != lastCanDiag1d4TxData[bi]) {
                    tx1d4ChangeMask |= static_cast<uint8_t>(1U << bi);
                }
            }
            if (tx1d4Len != lastCanDiag1d4TxLen) {
                tx1d4ChangeMask = static_cast<uint8_t>((tx1d4Len >= 8U) ? 0xFFU : ((1U << tx1d4Len) - 1U));
            }

            uint8_t tx11aChangeMask = 0U;
            const uint8_t tx11aLen = canStats.last11aTxLen;
            const uint8_t tx11aCmpLen = (tx11aLen < lastCanDiag11aTxLen) ? tx11aLen : lastCanDiag11aTxLen;
            for (uint8_t bi = 0U; bi < tx11aCmpLen; ++bi) {
                if (canStats.last11aTxData[bi] != lastCanDiag11aTxData[bi]) {
                    tx11aChangeMask |= static_cast<uint8_t>(1U << bi);
                }
            }
            if (tx11aLen != lastCanDiag11aTxLen) {
                tx11aChangeMask = static_cast<uint8_t>((tx11aLen >= 8U) ? 0xFFU : ((1U << tx11aLen) - 1U));
            }

            // Log CAN-EVENT to USB for diagnostics (always), but to telnet only on errors
            Serial.printf("[CAN-EVENT] ready=%d state=%u tx_total=%lu tx_1d4=%lu tx_11a=%lu tx1d4_chg=0x%02X tx11a_chg=0x%02X tx1d4_b6=0x%02X tx1d4_b7=0x%02X tx11a_b6=0x%02X tx11a_b7=0x%02X tx_fail=%lu tx_not_ready=%lu recov=%lu bus_off=%lu status_q_fail=%lu twai(rxq=%lu txq=%lu rx_miss=%lu rx_ovr=%lu arb_lost=%lu bus_err=%lu tec=%lu rec=%lu)\n",
                          canStats.ready ? 1 : 0,
                          static_cast<unsigned>(canStats.lastTwaiState),
                          static_cast<unsigned long>(canStats.txFrames),
                          static_cast<unsigned long>(canStats.tx1d4Frames),
                          static_cast<unsigned long>(canStats.tx11aFrames),
                          static_cast<unsigned>(tx1d4ChangeMask),
                          static_cast<unsigned>(tx11aChangeMask),
                          static_cast<unsigned>(canStats.last1d4TxLen > 6U ? canStats.last1d4TxData[6] : 0U),
                          static_cast<unsigned>(canStats.last1d4TxLen > 7U ? canStats.last1d4TxData[7] : 0U),
                          static_cast<unsigned>(canStats.last11aTxLen > 6U ? canStats.last11aTxData[6] : 0U),
                          static_cast<unsigned>(canStats.last11aTxLen > 7U ? canStats.last11aTxData[7] : 0U),
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
            
            // Forward to telnet and Serial0 ONLY if there's an error condition
            if (canStats.txFailures > 0 || canStats.txWhileNotReady > 0 || canStats.recoveries > 0 ||
                canStats.busOffEvents > 0 || canStats.statusQueryFailures > 0 || canStats.twaiBusError > 0 ||
                canStats.twaiTxErrorCounter > 0 || canStats.twaiRxErrorCounter > 0) {
                MetaSense::TelnetSerialBridge::telnetBridgePrintf("[CAN-EVENT-ERROR] ready=%d state=%u tx_total=%lu tx_1d4=%lu tx_11a=%lu tx_fail=%lu tx_not_ready=%lu recov=%lu bus_off=%lu status_q_fail=%lu twai(rx_miss=%lu rx_ovr=%lu arb_lost=%lu bus_err=%lu tec=%lu rec=%lu)\n",
                          canStats.ready ? 1 : 0,
                          static_cast<unsigned>(canStats.lastTwaiState),
                          static_cast<unsigned long>(canStats.txFrames),
                          static_cast<unsigned long>(canStats.tx1d4Frames),
                          static_cast<unsigned long>(canStats.tx11aFrames),
                          static_cast<unsigned long>(canStats.txFailures),
                          static_cast<unsigned long>(canStats.txWhileNotReady),
                          static_cast<unsigned long>(canStats.recoveries),
                          static_cast<unsigned long>(canStats.busOffEvents),
                          static_cast<unsigned long>(canStats.statusQueryFailures),
                          static_cast<unsigned long>(canStats.twaiRxMissed),
                          static_cast<unsigned long>(canStats.twaiRxOverrun),
                          static_cast<unsigned long>(canStats.twaiArbLost),
                          static_cast<unsigned long>(canStats.twaiBusError),
                          static_cast<unsigned long>(canStats.twaiTxErrorCounter),
                          static_cast<unsigned long>(canStats.twaiRxErrorCounter));
                Serial0.printf("[CAN-EVENT-ERROR] ready=%d state=%u tx_total=%lu tx_1d4=%lu tx_11a=%lu tx_fail=%lu tx_not_ready=%lu recov=%lu bus_off=%lu status_q_fail=%lu twai(rx_miss=%lu rx_ovr=%lu arb_lost=%lu bus_err=%lu tec=%lu rec=%lu)\n",
                          canStats.ready ? 1 : 0,
                          static_cast<unsigned>(canStats.lastTwaiState),
                          static_cast<unsigned long>(canStats.txFrames),
                          static_cast<unsigned long>(canStats.tx1d4Frames),
                          static_cast<unsigned long>(canStats.tx11aFrames),
                          static_cast<unsigned long>(canStats.txFailures),
                          static_cast<unsigned long>(canStats.txWhileNotReady),
                          static_cast<unsigned long>(canStats.recoveries),
                          static_cast<unsigned long>(canStats.busOffEvents),
                          static_cast<unsigned long>(canStats.statusQueryFailures),
                          static_cast<unsigned long>(canStats.twaiRxMissed),
                          static_cast<unsigned long>(canStats.twaiRxOverrun),
                          static_cast<unsigned long>(canStats.twaiArbLost),
                          static_cast<unsigned long>(canStats.twaiBusError),
                          static_cast<unsigned long>(canStats.twaiTxErrorCounter),
                          static_cast<unsigned long>(canStats.twaiRxErrorCounter));
            }
            lastCanEventLogMs = now;
        }

        lastCanDiagInitialized = true;
        lastCanDiagReady = canStats.ready;
        lastCanDiagState = canStats.lastTwaiState;
        lastCanDiagTxFrames = canStats.txFrames;
        lastCanDiagTx1d4Frames = canStats.tx1d4Frames;
        lastCanDiagTx11aFrames = canStats.tx11aFrames;
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
        lastCanDiag1d4TxLen = canStats.last1d4TxLen;
        lastCanDiag11aTxLen = canStats.last11aTxLen;
        memset(lastCanDiag1d4TxData, 0, sizeof(lastCanDiag1d4TxData));
        memset(lastCanDiag11aTxData, 0, sizeof(lastCanDiag11aTxData));
        if (lastCanDiag1d4TxLen > 0U) {
            memcpy(lastCanDiag1d4TxData, canStats.last1d4TxData,
                   (lastCanDiag1d4TxLen <= sizeof(lastCanDiag1d4TxData)) ? lastCanDiag1d4TxLen : sizeof(lastCanDiag1d4TxData));
        }
        if (lastCanDiag11aTxLen > 0U) {
            memcpy(lastCanDiag11aTxData, canStats.last11aTxData,
                   (lastCanDiag11aTxLen <= sizeof(lastCanDiag11aTxData)) ? lastCanDiag11aTxLen : sizeof(lastCanDiag11aTxData));
        }

        if (s_leaf1d4FreshLastLogMs == 0U || (now - s_leaf1d4FreshLastLogMs) >= 1000U) {
            uint8_t freshMask = 0U;
            const uint8_t currLen = canStats.last1d4TxLen;
            const uint8_t cmpLen = (currLen < s_leaf1d4FreshPrevLen) ? currLen : s_leaf1d4FreshPrevLen;
            for (uint8_t bi = 0U; bi < cmpLen; ++bi) {
                if (canStats.last1d4TxData[bi] != s_leaf1d4FreshPrevData[bi]) {
                    freshMask |= static_cast<uint8_t>(1U << bi);
                }
            }
            if (currLen != s_leaf1d4FreshPrevLen) {
                freshMask = static_cast<uint8_t>((currLen >= 8U) ? 0xFFU : ((1U << currLen) - 1U));
            }

            if (freshMask == 0U) {
                ++s_leaf1d4FreshStaticRun;
            } else {
                s_leaf1d4FreshStaticRun = 0U;
            }

            const Leaf1d4CommandDecode txDecoded = decodeLeaf1d4Command(canStats.last1d4TxData, currLen);
            const bool txSemanticMatch = txDecoded.valid &&
                                         (txDecoded.motorAmpTorqueRaw == s_leaf1d4PayloadTorqueRaw) &&
                                         (txDecoded.hvSupplyStatus == s_leaf1d4PayloadHvStatus) &&
                                         (txDecoded.relayPlusStatus == s_leaf1d4PayloadRelayPlus) &&
                                         (txDecoded.chargeStatus == s_leaf1d4PayloadChargeStatus) &&
                                         (txDecoded.hcmClock == s_leaf1d4PayloadClock) &&
                                         (txDecoded.crc1d4 == s_leaf1d4PayloadCrc) &&
                                         (s_leaf1d4PayloadCrc == s_leaf1d4PayloadCrcCalc);

            const uint32_t tx1d4Count = canStats.tx1d4Frames;
            const uint32_t tx1d4Delta = tx1d4Count - s_leaf1d4FreshLastTxCount;
            const float tx1d4Hz = (s_leaf1d4FreshLastLogMs == 0U)
                ? 0.0f
                : (1000.0f * static_cast<float>(tx1d4Delta) /
                   static_cast<float>(now - s_leaf1d4FreshLastLogMs));
            const bool haveFull1d4 = currLen >= 8U;
            const uint8_t crcCalc = haveFull1d4 ? computeLeaf1d4CrcConformant(canStats.last1d4TxData) : 0U;
            const uint8_t crcTx = haveFull1d4 ? canStats.last1d4TxData[7] : 0U;

            Serial.printf("[VCM-1D4-TX-SELF] n=%lu sem=%u tq=%.2f raw=%d hv=%u rplus=%u clk=%u charge=%u crc=0x%02X calc=0x%02X raw=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                          static_cast<unsigned long>(tx1d4Count),
                          txSemanticMatch ? 1U : 0U,
                          txDecoded.motorAmpTorqueNm,
                          static_cast<int>(txDecoded.motorAmpTorqueRaw),
                          txDecoded.hvSupplyStatus ? 1U : 0U,
                          txDecoded.relayPlusStatus ? 1U : 0U,
                          static_cast<unsigned>(txDecoded.hcmClock),
                          static_cast<unsigned>(txDecoded.chargeStatus),
                          static_cast<unsigned>(txDecoded.crc1d4),
                          static_cast<unsigned>(computeLeaf1d4CrcConformant(canStats.last1d4TxData)),
                          static_cast<unsigned>(currLen > 0U ? canStats.last1d4TxData[0] : 0U),
                          static_cast<unsigned>(currLen > 1U ? canStats.last1d4TxData[1] : 0U),
                          static_cast<unsigned>(currLen > 2U ? canStats.last1d4TxData[2] : 0U),
                          static_cast<unsigned>(currLen > 3U ? canStats.last1d4TxData[3] : 0U),
                          static_cast<unsigned>(currLen > 4U ? canStats.last1d4TxData[4] : 0U),
                          static_cast<unsigned>(currLen > 5U ? canStats.last1d4TxData[5] : 0U),
                          static_cast<unsigned>(currLen > 6U ? canStats.last1d4TxData[6] : 0U),
                          static_cast<unsigned>(currLen > 7U ? canStats.last1d4TxData[7] : 0U));

            Serial.printf("[VCM-1D4-TX-FRESH] n=%lu d=%lu hz=%.1f chg=0x%02X static_run=%lu b2=0x%02X b6=0x%02X b7=0x%02X crc_ok=%u\n",
                          static_cast<unsigned long>(tx1d4Count),
                          static_cast<unsigned long>(tx1d4Delta),
                          tx1d4Hz,
                          static_cast<unsigned>(freshMask),
                          static_cast<unsigned long>(s_leaf1d4FreshStaticRun),
                          static_cast<unsigned>(currLen > 2U ? canStats.last1d4TxData[2] : 0U),
                          static_cast<unsigned>(currLen > 6U ? canStats.last1d4TxData[6] : 0U),
                          static_cast<unsigned>(currLen > 7U ? canStats.last1d4TxData[7] : 0U),
                          static_cast<unsigned>(haveFull1d4 && (crcTx == crcCalc) ? 1U : 0U));

            s_leaf1d4FreshLastLogMs = now;
            s_leaf1d4FreshLastTxCount = tx1d4Count;
            s_leaf1d4FreshPrevLen = currLen;
            memset(s_leaf1d4FreshPrevData, 0, sizeof(s_leaf1d4FreshPrevData));
            if (currLen > 0U) {
                memcpy(s_leaf1d4FreshPrevData,
                       canStats.last1d4TxData,
                       (currLen <= sizeof(s_leaf1d4FreshPrevData)) ? currLen : sizeof(s_leaf1d4FreshPrevData));
            }
        }
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
    
    // Yield to allow network/WiFi task to run
    taskYIELD();
    
    // Serial heartbeat (0.2Hz = every 5s) for diagnostics
    static uint32_t lastHeartbeatMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastHeartbeatMs >= 5000U) {
        lastHeartbeatMs = nowMs;
        IPAddress ip = WiFi.localIP();
        int32_t rssi = WiFi.RSSI();
        Serial.printf("[HEARTBEAT] RPM=%.0f Leaf_RPM=%.0f Temps: Inv=%.1f Stator=%.1f Coolant=%.1f IP=%d.%d.%d.%d RSSI=%ld dBm ms=%lu\n",
                      tele.rpm, tele.leaf_rpm, tele.leaf_invTempC, tele.leaf_statorTempC, tele.leaf_coolantTempC,
                      ip[0], ip[1], ip[2], ip[3], rssi, nowMs);
        // Forward to telnet clients
        MetaSense::TelnetSerialBridge::telnetBridgePrintf("[HEARTBEAT] RPM=%.0f Leaf_RPM=%.0f Temps: Inv=%.1f Stator=%.1f Coolant=%.1f IP=%d.%d.%d.%d RSSI=%ld dBm ms=%lu\n",
                      tele.rpm, tele.leaf_rpm, tele.leaf_invTempC, tele.leaf_statorTempC, tele.leaf_coolantTempC,
                      ip[0], ip[1], ip[2], ip[3], rssi, nowMs);
        
        // Print 1D4 frame debug info (read directly from cached frame - simple Serial.print approach)
        static uint32_t lastPrintedTxCount = 0xFFFFFFFFUL;
        // Print every 10 frames to avoid sync issues with core-1 counter updates
        if ((s_leaf1d4TxFrameCount / 10U) != (lastPrintedTxCount / 10U)) {
            lastPrintedTxCount = s_leaf1d4TxFrameCount;
            // Extract HCM_CLOCK from bits 38-39 (byte 4, bits 6-7)
            const uint8_t hcmClock = (s_leaf1d4PayloadCachedFrameData[4] >> 6) & 0x03U;
            Serial.print("[1D4-");
            Serial.print(s_leaf1d4TxFrameCount);
            Serial.print("] C=");
            Serial.print(hcmClock);
            Serial.print(" CRC=");
            Serial.print(s_leaf1d4PayloadCachedFrameData[7], HEX);
            Serial.print(" Data:");
            for (int i = 0; i < 8; i++) {
                Serial.print(" ");
                if (s_leaf1d4PayloadCachedFrameData[i] < 0x10) Serial.print("0");
                Serial.print(s_leaf1d4PayloadCachedFrameData[i], HEX);
            }
            Serial.println();
        }
    }
}

void publish()
{
    publishTelemetry();
}

} // namespace MetaSense::Input
