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
static uint8_t s_leafLast120ShadowData[4] = {0U, 0U, 0U, 0U};
static uint32_t s_leafLast120ShadowMs = 0;
static float s_leaf120PayloadTorqueNm = 0.0f;
static int16_t s_leaf120PayloadTorqueRaw = 0;
static uint32_t s_leaf120PayloadMs = 0;
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
static uint32_t leaf120B2PosCounts[256] = {0};
static uint32_t leaf120B2NegCounts[256] = {0};
static uint32_t leaf120B2ZeroCounts[256] = {0};
static uint32_t leaf120B2BrakeCounts[256] = {0};
static uint32_t leaf120B2PosSamples = 0;
static uint32_t leaf120B2NegSamples = 0;
static uint32_t leaf120B2ZeroSamples = 0;
static uint32_t leaf120B2BrakeSamples = 0;
constexpr uint8_t kLeaf120CrcCandidateCount = 17U;
static uint32_t leaf120CrcCandidateMatches[kLeaf120CrcCandidateCount] = {0};
static uint32_t leaf120CrcSamples = 0;
static uint32_t leaf120CrcBrakeSamples[2] = {0U, 0U};
static uint32_t leaf120Ref00StateCorrectedMatchesByBrake[2] = {0U, 0U};
static uint32_t leaf120Ref00Fixed8MatchesByBrake[2] = {0U, 0U};
static uint32_t leaf120CrcRefIdLoMatchesByBrake[2] = {0U, 0U};
static uint32_t leaf120CrcCmdStateSamples[3] = {0U, 0U, 0U};
static uint32_t leaf120Ref00StateCorrectedMatchesByCmdState[3] = {0U, 0U, 0U};
static uint32_t leaf120Ref00Fixed8MatchesByCmdState[3] = {0U, 0U, 0U};
static uint32_t leaf120CrcRefIdLoMatchesByCmdState[3] = {0U, 0U, 0U};
static uint32_t leaf120CrcMotorHiNibSamples[16] = {0U};
static uint32_t leaf120Ref00StateCorrectedMatchesByMotorHiNib[16] = {0U};
static uint32_t leaf120Ref00Fixed8MatchesByMotorHiNib[16] = {0U};
static uint32_t leaf120CrcRefIdLoMatchesByMotorHiNib[16] = {0U};
constexpr uint8_t kLeaf120MotorScopeCount = 4U;
static uint32_t leaf120MotorScopeSamples[kLeaf120MotorScopeCount] = {0U};
static uint32_t leaf120MotorScopeDirectMatches[kLeaf120MotorScopeCount] = {0U};
static uint32_t leaf120MotorScopeStateFixMatches[kLeaf120MotorScopeCount] = {0U};
static uint32_t leaf120MotorScopeResidueByState[kLeaf120MotorScopeCount][16][256] = {{{0U}}};
static uint8_t leaf120MotorScopeTopResidueByState[kLeaf120MotorScopeCount][16] = {{0U}};
static uint32_t leaf120MotorScopeTopResidueCountByState[kLeaf120MotorScopeCount][16] = {{0U}};
static uint32_t leaf120CrcNibbleSamples[16] = {0U};
static uint32_t leaf120Ref00StateCorrectedMatchesByNibble[16] = {0U};
static uint32_t leaf120Ref00Fixed8MatchesByNibble[16] = {0U};
static uint32_t leaf120CrcRefIdLoMatchesByNibble[16] = {0U};
static const uint8_t kLeaf120Ref00ResidueByState8[8] = {
    0x00U, 0xE1U, 0x47U, 0xA6U, 0x7AU, 0x9BU, 0x3DU, 0xDCU
};
static uint32_t leaf120Ref00ResidueByState[16][256] = {{0}};
static uint8_t leaf120Ref00TopResidueByState[16] = {0U};
static uint32_t leaf120Ref00TopResidueCountByState[16] = {0U};
static uint32_t leaf120Ref00StateCorrectedMatches = 0U;
static uint32_t leaf120Ref00Fixed8Matches = 0U;
static uint32_t leaf120TxExpSamples = 0U;
static uint32_t leaf120TxExpExactCrcMatches = 0U;
static uint32_t leaf120TxExpHiCtrCrcMatches = 0U;
static uint32_t leaf120TxExpLoCtrCrcMatches = 0U;
static uint32_t leaf120TxExpHiCtrB2Matches = 0U;
static uint32_t leaf120TxExpLoCtrB2Matches = 0U;
constexpr uint8_t kLeaf1daCrcCandidateCount = 17U;
static uint32_t leaf1daCrcCandidateMatches[kLeaf1daCrcCandidateCount] = {0};
static uint32_t leaf1daCrcSamples = 0;
constexpr uint8_t kLeaf1daResidueCandidateCount = 8U;
static uint32_t leaf1daResidueCounts[kLeaf1daResidueCandidateCount][256] = {{0}};
static uint32_t leaf1daResidueByClock[kLeaf1daResidueCandidateCount][4][256] = {{{0}}};
static uint32_t leaf1daClockSamples[4] = {0};
static uint32_t leaf1daIdLoResidueByClock[4][256] = {{0}};
static uint8_t leaf1daIdLoTopResidueByClock[4] = {0U, 0U, 0U, 0U};
static uint32_t leaf1daIdLoTopResidueCountByClock[4] = {0U, 0U, 0U, 0U};
static uint32_t leaf1daIdLoClockCorrectedMatches = 0U;
static uint32_t leaf1daResidueSamples = 0;
constexpr uint8_t kLeaf1daAutosarCandidateCount = 4U;
static uint32_t leaf1daAutosarCandidateMatches[kLeaf1daAutosarCandidateCount] = {0};
static uint32_t leaf1daAutosarSamples = 0;
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
static const uint32_t CAN_ALT_DIAG_LOG_PERIOD_MS = 2000;
static const uint32_t CAN_PRE_DIAG_LOG_PERIOD_MS = 2000;
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
#ifndef METASENSE_LEAF_120_CMD_BASE_SLOPE
#define METASENSE_LEAF_120_CMD_BASE_SLOPE 0.0300f
#endif
#ifndef METASENSE_LEAF_120_CMD_BASE_OFFSET_NM
#define METASENSE_LEAF_120_CMD_BASE_OFFSET_NM 0.030f
#endif
#ifndef METASENSE_LEAF_120_CMD_MIN_FIT_SAMPLES
#define METASENSE_LEAF_120_CMD_MIN_FIT_SAMPLES 128U
#endif
#ifndef METASENSE_LEAF_VCM_LEGACY_VERBOSE_LOGS
#define METASENSE_LEAF_VCM_LEGACY_VERBOSE_LOGS 0
#endif
#ifndef METASENSE_LEAF_VCM_RX_WARN_LOGS
#define METASENSE_LEAF_VCM_RX_WARN_LOGS 0
#endif
#ifndef METASENSE_LEAF_CRC_DEEP_LOGS
#define METASENSE_LEAF_CRC_DEEP_LOGS 0
#endif
#ifndef METASENSE_LEAF_MONITOR_SIMPLE_LOGS
#define METASENSE_LEAF_MONITOR_SIMPLE_LOGS 1
#endif
#ifndef METASENSE_LEAF_MONITOR_SIMPLE_DECIMATE
#define METASENSE_LEAF_MONITOR_SIMPLE_DECIMATE 20U
#endif
#ifndef METASENSE_LEAF_120_AUX_LOGS
#define METASENSE_LEAF_120_AUX_LOGS 0
#endif
#ifndef METASENSE_LEAF_120_SHADOW_LOGS
#define METASENSE_LEAF_120_SHADOW_LOGS 1
#endif
#ifndef METASENSE_LEAF_120_COMPARE_COPY_RX
#define METASENSE_LEAF_120_COMPARE_COPY_RX 0
#endif
#ifndef METASENSE_LEAF_120_SHADOW_STATE_STRATEGY
// 0: use explicit brake/gear inputs from control path
// 1: derive state from torque sign (+ = MOTOR, - = BRAKE)
// 2: 4-quadrant model using torque sign and RPM sign
//    (0Nm=Neutral, +Nm=Forward, -Nm with rpm>0=Brake, -Nm with rpm<0=Reverse)
#define METASENSE_LEAF_120_SHADOW_STATE_STRATEGY 0
#endif
#ifndef METASENSE_LEAF_120_SHADOW_TORQUE_DEADBAND_NM
#define METASENSE_LEAF_120_SHADOW_TORQUE_DEADBAND_NM 1.0f
#endif
#ifndef METASENSE_LEAF_120_SHADOW_NEUTRAL_STARTUP_MS
// >0 keeps state in neutral for startup transient window before forcing F states.
#define METASENSE_LEAF_120_SHADOW_NEUTRAL_STARTUP_MS 0U
#endif
#ifndef METASENSE_LEAF_120_SHADOW_IDLE_FORCE_FWD
// In torque-sign strategy, keep F in near-zero torque zone when enabled.
#define METASENSE_LEAF_120_SHADOW_IDLE_FORCE_FWD 1
#endif
#ifndef METASENSE_LEAF_120_SHADOW_REVERSE_RPM_DEADBAND
#define METASENSE_LEAF_120_SHADOW_REVERSE_RPM_DEADBAND 30.0f
#endif
#ifndef METASENSE_LEAF_120_FACTS_LOGS
#define METASENSE_LEAF_120_FACTS_LOGS 0
#endif
#ifndef METASENSE_LEAF_120_FACTS_LOG_PERIOD_MS
#define METASENSE_LEAF_120_FACTS_LOG_PERIOD_MS 5000U
#endif
#ifndef METASENSE_LEAF_120_NOISE_LOGS
#define METASENSE_LEAF_120_NOISE_LOGS 0
#endif
#ifndef METASENSE_LEAF_120_NOISE_DB_NM
#define METASENSE_LEAF_120_NOISE_DB_NM 0.4f
#endif
#ifndef METASENSE_LEAF_120_NOISE_EMA_ALPHA
#define METASENSE_LEAF_120_NOISE_EMA_ALPHA 0.10f
#endif
#ifndef METASENSE_LEAF_120_NOISE_OUTLIER_ABS_NM
#define METASENSE_LEAF_120_NOISE_OUTLIER_ABS_NM 5.0f
#endif
#ifndef METASENSE_LEAF_120_DT_SLOPE_STEADY_NMPS
#define METASENSE_LEAF_120_DT_SLOPE_STEADY_NMPS 10.0f
#endif
#ifndef METASENSE_LEAF_120_DT_SLOPE_HIGH_NMPS
#define METASENSE_LEAF_120_DT_SLOPE_HIGH_NMPS 120.0f
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
// Default behavior: prefer 0x1DA-derived RPM when fresh, else fallback to tachogen.
// Set to 1 only for forced tachogen-only diagnostics.
#define METASENSE_FORCE_TACHO_RPM_SOURCE 0
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

struct Leaf120CommandDecode {
    uint16_t torqueDemandRawBe = 0U;
    int16_t torqueDemandSignedBe = 0;
    float torqueDemandNmBase = 0.0f;
    uint8_t unknown120_2 = 0U;
    uint8_t crc120 = 0U;
};

Leaf120CommandDecode decodeLeaf120Command(const uint8_t* data, uint8_t len)
{
    Leaf120CommandDecode decoded;
    if (data == nullptr || len == 0U) {
        return decoded;
    }

    if (len >= 2U) {
        decoded.torqueDemandRawBe = (static_cast<uint16_t>(data[0]) << 8) |
                                    static_cast<uint16_t>(data[1]);
        decoded.torqueDemandSignedBe = static_cast<int16_t>(decoded.torqueDemandRawBe);
        decoded.torqueDemandNmBase = METASENSE_LEAF_120_CMD_BASE_OFFSET_NM +
                                     (METASENSE_LEAF_120_CMD_BASE_SLOPE * static_cast<float>(decoded.torqueDemandSignedBe));
    }
    if (len >= 3U) {
        decoded.unknown120_2 = data[2];
    }
    if (len >= 4U) {
        decoded.crc120 = data[3];
    }

    return decoded;
}

#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
struct Leaf120TopValue {
    uint8_t value = 0U;
    uint32_t count = 0U;
};

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
#endif

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
#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
            Serial.println("[VCM-HS] First 0x1DA observed, starting extended 0x120 handshake campaign");
            Serial0.println("[VCM-HS] First 0x1DA observed, starting extended 0x120 handshake campaign");
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
        // Always emit source/fallback state so reconnecting UIs cannot keep stale defaults.
        json += "\"can_fallback\":" + String(canFallbackActive ? 1 : 0) + ",";
        prevCanFallbackActive = canFallbackActive;

        String rpmSourceStr = String(activeRpmFromCan ? "leafrpm" : "tachogen");
        json += "\"rpm_source_active\":\"" + rpmSourceStr + "\",";
        prevRpmSourceCanSent = activeRpmFromCan;
        prevRpmSourceCanInitialized = true;
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
        json += "\"leaf_id1da_frames\":" + String(static_cast<unsigned long>(leafFbDiag.rpm_frames)) + ",";
        json += "\"leaf_id1da_age_ms\":" + String(static_cast<unsigned long>(elapsedMsSafe(nowMs, leafFbDiag.rpm_update_ms))) + ",";
        {
            const Leaf120CommandDecode id120TxUi = decodeLeaf120Command(s_leafLast120ShadowData,
                                                                         sizeof(s_leafLast120ShadowData));
            const unsigned long id120TxAgeMsUi = static_cast<unsigned long>(elapsedMsSafe(nowMs, s_leafLast120ShadowMs));
            const bool id120TxValidUi = (s_leafLast120ShadowMs != 0U) &&
                                        (id120TxAgeMsUi <= static_cast<unsigned long>(CAN_TEMP_TIMEOUT_MS));
            const Leaf120CommandDecode id120SelectedUi = id120TxValidUi ? id120TxUi : Leaf120CommandDecode{};
            const bool payloadVarFreshUi = (s_leaf120PayloadMs != 0U) &&
                                           (elapsedMsSafe(nowMs, s_leaf120PayloadMs) <= static_cast<unsigned long>(CAN_TEMP_TIMEOUT_MS));
            const float id120DisplayNmUi = payloadVarFreshUi ? s_leaf120PayloadTorqueNm : 0.0f;
            const char* id120DisplaySourceUi = "payload_var";

            // Canonical 0x120 command fields (TX context, not RX acceptance).
            json += "\"leaf_120_cmd_nm\":" + String(id120DisplayNmUi, 2) + ",";
            json += "\"leaf_120_cmd_raw\":" + String(static_cast<int>(payloadVarFreshUi ? s_leaf120PayloadTorqueRaw : 0)) + ",";
            json += "\"leaf_120_unknown_2\":" + String(static_cast<unsigned>(id120TxValidUi ? id120SelectedUi.unknown120_2 : 0U)) + ",";
            json += "\"leaf_120_crc\":" + String(static_cast<unsigned>(id120TxValidUi ? id120SelectedUi.crc120 : 0U)) + ",";
            json += "\"leaf_120_tx_age_ms\":" + String(id120TxAgeMsUi) + ",";
            json += "\"leaf_120_cmd_source\":\"" + String(id120DisplaySourceUi) + "\",";
            json += "\"leaf_120_tx_cmd_nm\":" + String(id120TxValidUi ? id120TxUi.torqueDemandNmBase : 0.0f, 2) + ",";
            json += "\"leaf_120_tx_cmd_raw\":" + String(static_cast<int>(id120TxValidUi ? id120TxUi.torqueDemandSignedBe : 0)) + ",";
            json += "\"leaf_120_tx_unknown_2\":" + String(static_cast<unsigned>(id120TxValidUi ? id120TxUi.unknown120_2 : 0U)) + ",";
            json += "\"leaf_120_tx_crc\":" + String(static_cast<unsigned>(id120TxValidUi ? id120TxUi.crc120 : 0U)) + ",";
            json += "\"leaf_120_tx_b0\":" + String(static_cast<unsigned>(id120TxValidUi ? s_leafLast120ShadowData[0] : 0U)) + ",";
            json += "\"leaf_120_tx_b1\":" + String(static_cast<unsigned>(id120TxValidUi ? s_leafLast120ShadowData[1] : 0U)) + ",";
            json += "\"leaf_120_tx_b2\":" + String(static_cast<unsigned>(id120TxValidUi ? s_leafLast120ShadowData[2] : 0U)) + ",";
            json += "\"leaf_120_tx_b3\":" + String(static_cast<unsigned>(id120TxValidUi ? s_leafLast120ShadowData[3] : 0U)) + ",";
            json += "\"leaf_120_tx_b4\":" + String(static_cast<unsigned>(id120TxValidUi ? s_leafLast120ShadowData[4] : 0U)) + ",";
            json += "\"leaf_120_tx_b5\":" + String(static_cast<unsigned>(id120TxValidUi ? s_leafLast120ShadowData[5] : 0U)) + ",";
            json += "\"leaf_120_tx_b6\":" + String(static_cast<unsigned>(id120TxValidUi ? s_leafLast120ShadowData[6] : 0U)) + ",";
            json += "\"leaf_120_tx_b7\":" + String(static_cast<unsigned>(id120TxValidUi ? s_leafLast120ShadowData[7] : 0U)) + ",";
        }
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
        json += "\"hw_start_gate\":\"" + String(MetaSense::HardwareOutputStateMachine::startGateReason()) + "\",";
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

void buildLeaf120ShadowFrame(float torqueDemandNm,
                             bool readyBit,
                             bool hvOkBit,
                             bool brakeBit,
                             bool gearDriveBit,
                             uint8_t (&out)[4]);

bool sendLeafTorqueCommand120(float torqueDemandNm,
                              bool readyBit,
                              bool hvOkBit,
                              bool brakeBit,
                              bool gearDriveBit)
{
    const float torqueClamped = constrain(torqueDemandNm, -300.0f, 300.0f);
    const int16_t torqueRaw = static_cast<int16_t>(lroundf(torqueClamped * 10.0f));
    const uint32_t nowMs = millis();

    // Canonical monitor value: this is the actual torque payload component
    // used when constructing the outbound 0x120 frame.
    s_leaf120PayloadTorqueNm = torqueClamped;
    s_leaf120PayloadTorqueRaw = torqueRaw;
    s_leaf120PayloadMs = nowMs;

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

    const uint16_t checksum = static_cast<uint16_t>(data[0]) +
                              static_cast<uint16_t>(data[1]) +
                              static_cast<uint16_t>(data[2]) +
                              static_cast<uint16_t>(data[3]) +
                              static_cast<uint16_t>(data[4]) +
                              static_cast<uint16_t>(data[6]);
    data[7] = static_cast<uint8_t>(checksum & 0xFF);

    rollingCounter = static_cast<uint8_t>((rollingCounter + 1) & 0x0F);
    const bool sent = MetaSense::CANBus::send(0x120, data, 8);
    if (sent) {
        const float previousCmd = s_leafLastSentTorqueNm;
        s_leafLastSentTorqueNm = torqueClamped;
        s_leafLastSentTorqueMs = nowMs;
        float shadowTorqueNm = torqueClamped;
#if METASENSE_LEAF_120_COMPARE_COPY_RX
        {
            const MetaSense::CANBus::Stats& canStats = MetaSense::CANBus::stats();
            const bool rx120Fresh = (canStats.last120Len >= 4U) &&
                                    (elapsedMsSafe(nowMs, canStats.last120Ms) <= CAN_TEMP_TIMEOUT_MS);
            if (rx120Fresh) {
                const Leaf120CommandDecode rx120 = decodeLeaf120Command(canStats.last120Data,
                                                                         canStats.last120Len);
                shadowTorqueNm = rx120.torqueDemandNmBase;
            }
        }
#endif
        buildLeaf120ShadowFrame(shadowTorqueNm,
                                readyBit,
                                hvOkBit,
                                brakeBit,
                                gearDriveBit,
                                s_leafLast120ShadowData);
        s_leafLast120ShadowMs = nowMs;

        if (fabsf(torqueClamped - previousCmd) >= kLeafTorqueTrackStepNm) {
            s_leafTorqueTrackStartMs = nowMs;
            s_leafTorqueTrackTargetNm = torqueClamped;
            s_leafTorqueTrackPending = true;
        }
    }

    return sent;
}

bool sendLeafTorqueCommand55B(float torqueDemandNm,
                              bool readyBit,
                              bool hvOkBit,
                              bool brakeBit,
                              bool gearDriveBit)
{
    return sendLeafTorqueCommand120(torqueDemandNm,
                                    readyBit,
                                    hvOkBit,
                                    brakeBit,
                                    gearDriveBit);
}

uint8_t crc8Lsb120(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00U;
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            const bool lsb = (crc & 0x01U) != 0U;
            crc >>= 1;
            if (lsb) {
                crc ^= 0xB8U;
            }
        }
    }
    return crc;
}

enum class Leaf120ShadowStateClass : uint8_t {
    Neutral = 0,
    Idle = 1,
    Motor = 2,
    Brake = 3,
    Reverse = 4,
};

const char* leaf120ShadowStateClassName(Leaf120ShadowStateClass stateClass)
{
    switch (stateClass) {
    case Leaf120ShadowStateClass::Neutral:
        return "NEUTRAL";
    case Leaf120ShadowStateClass::Idle:
        return "IDLE";
    case Leaf120ShadowStateClass::Motor:
        return "MOTOR";
    case Leaf120ShadowStateClass::Brake:
        return "BRAKE";
    case Leaf120ShadowStateClass::Reverse:
        return "REVERSE";
    }
    return "IDLE";
}

constexpr uint8_t kLeaf120StateClassCount = 5U;
constexpr uint8_t kLeaf120SlopeClassCount = 4U;

uint8_t leaf120ShadowStateClassIndex(Leaf120ShadowStateClass stateClass)
{
    switch (stateClass) {
    case Leaf120ShadowStateClass::Neutral:
        return 0U;
    case Leaf120ShadowStateClass::Idle:
        return 1U;
    case Leaf120ShadowStateClass::Motor:
        return 2U;
    case Leaf120ShadowStateClass::Brake:
        return 3U;
    case Leaf120ShadowStateClass::Reverse:
        return 4U;
    }
    return 1U;
}

uint8_t topByteFromCounts(const uint32_t* counts, uint32_t& outCount)
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

const char* leaf120ShadowStrategyName()
{
#if METASENSE_LEAF_120_SHADOW_STATE_STRATEGY == 1
    return "torque_sign";
#elif METASENSE_LEAF_120_SHADOW_STATE_STRATEGY == 2
    return "quadrant";
#else
    return "brake_flag";
#endif
}

Leaf120ShadowStateClass resolveLeaf120ShadowState(float torqueDemandNm,
                                                  float rpm,
                                                  bool brakeIn,
                                                  bool gearIn,
                                                  bool& brakeOut,
                                                  bool& gearOut)
{
    if (METASENSE_LEAF_120_SHADOW_NEUTRAL_STARTUP_MS > 0U &&
        millis() < METASENSE_LEAF_120_SHADOW_NEUTRAL_STARTUP_MS) {
        brakeOut = false;
        gearOut = false;
        return Leaf120ShadowStateClass::Neutral;
    }

    // Global neutral rule: zero/near-zero torque is a valid Neutral command.
    const float tqDeadband = METASENSE_LEAF_120_SHADOW_TORQUE_DEADBAND_NM;
    if (fabsf(torqueDemandNm) <= tqDeadband) {
        brakeOut = false;
        gearOut = false;
        return Leaf120ShadowStateClass::Neutral;
    }

#if METASENSE_LEAF_120_SHADOW_STATE_STRATEGY == 1
    if (torqueDemandNm > tqDeadband) {
        brakeOut = false;
        gearOut = true;
        return Leaf120ShadowStateClass::Motor;
    }
    if (torqueDemandNm < -tqDeadband) {
        brakeOut = true;
        gearOut = true;
        return Leaf120ShadowStateClass::Brake;
    }

    brakeOut = false;
    gearOut = false;
    return Leaf120ShadowStateClass::Neutral;
#elif METASENSE_LEAF_120_SHADOW_STATE_STRATEGY == 2
    const float rpmDeadband = METASENSE_LEAF_120_SHADOW_REVERSE_RPM_DEADBAND;

    if (torqueDemandNm > tqDeadband) {
        brakeOut = false;
        gearOut = true;
        return Leaf120ShadowStateClass::Motor;
    }

    // Negative torque branch.
    if (rpm < -rpmDeadband) {
        // Provisional reverse mapping candidate: BRAKE=1, GEAR=0.
        brakeOut = true;
        gearOut = false;
        return Leaf120ShadowStateClass::Reverse;
    }

    // Regen/brake branch when rpm is positive or near zero.
    brakeOut = true;
    gearOut = true;
    return Leaf120ShadowStateClass::Brake;
#else
    brakeOut = brakeIn;
    gearOut = gearIn;
    if (!gearOut) {
        return Leaf120ShadowStateClass::Neutral;
    }
    if (brakeOut) {
        return Leaf120ShadowStateClass::Brake;
    }

    return Leaf120ShadowStateClass::Motor;
#endif
}

void buildLeaf120ShadowFrame(float torqueDemandNm,
                             bool readyBit,
                             bool hvOkBit,
                             bool brakeBit,
                             bool gearDriveBit,
                             uint8_t (&out)[4])
{
    const float torqueClamped = constrain(torqueDemandNm, -300.0f, 300.0f);
    const float rawFloat = (torqueClamped - METASENSE_LEAF_120_CMD_BASE_OFFSET_NM) /
                           METASENSE_LEAF_120_CMD_BASE_SLOPE;
    long rawLong = lroundf(rawFloat);
    if (rawLong > 32767L) {
        rawLong = 32767L;
    } else if (rawLong < -32768L) {
        rawLong = -32768L;
    }
    const int16_t torqueRawBe = static_cast<int16_t>(rawLong);
    out[0] = static_cast<uint8_t>((static_cast<uint16_t>(torqueRawBe) >> 8) & 0xFFU);
    out[1] = static_cast<uint8_t>(static_cast<uint16_t>(torqueRawBe) & 0xFFU);

    bool shadowBrakeBit = brakeBit;
    bool shadowGearDriveBit = gearDriveBit;
    const float rpmForState = tele.rpm;
    const Leaf120ShadowStateClass stateClass = resolveLeaf120ShadowState(torqueClamped,
                                                                          rpmForState,
                                                                          brakeBit,
                                                                          gearDriveBit,
                                                                          shadowBrakeBit,
                                                                          shadowGearDriveBit);
    (void)stateClass;

    out[2] = static_cast<uint8_t>((readyBit ? 0x01U : 0x00U) |
                                  (hvOkBit ? 0x02U : 0x00U) |
                                  (shadowBrakeBit ? 0x04U : 0x00U) |
                                  (shadowGearDriveBit ? 0x08U : 0x00U));

    static const uint8_t k120ResidueByState8[8] = {
        0x00U, 0xE1U, 0x47U, 0xA6U, 0x7AU, 0x9BU, 0x3DU, 0xDCU
    };
    const uint8_t baseCrc = crc8Lsb120(out, 3U);
    out[3] = static_cast<uint8_t>(baseCrc ^ k120ResidueByState8[out[2] & 0x07U]);
}

void logLeaf120ShadowFrame(uint32_t nowMs,
                           float torqueDemandNm,
                           bool readyBit,
                           bool hvOkBit,
                           bool brakeBit,
                           bool gearDriveBit,
                           bool txSent)
{
#if METASENSE_LEAF_120_SHADOW_LOGS
    static uint32_t lastShadowLogMs = 0U;
    if (lastShadowLogMs != 0U && (nowMs - lastShadowLogMs) < 500U) {
        return;
    }
    lastShadowLogMs = nowMs;

    uint8_t shadow[4] = {0U, 0U, 0U, 0U};
    buildLeaf120ShadowFrame(torqueDemandNm, readyBit, hvOkBit, brakeBit, gearDriveBit, shadow);
    bool resolvedBrakeBit = brakeBit;
    bool resolvedGearDriveBit = gearDriveBit;
    const float rpmForState = tele.rpm;
    const Leaf120ShadowStateClass stateClass = resolveLeaf120ShadowState(torqueDemandNm,
                                                                          rpmForState,
                                                                          brakeBit,
                                                                          gearDriveBit,
                                                                          resolvedBrakeBit,
                                                                          resolvedGearDriveBit);

    const MetaSense::CANBus::Stats& canStats = MetaSense::CANBus::stats();
    const bool hasRx120 = canStats.last120Len >= 4U;
    const uint32_t rxAgeMs = elapsedMsSafe(nowMs, canStats.last120Ms);
    const bool freshRx120 = hasRx120 && (rxAgeMs <= CAN_RX_TARGET_MAX_AGE_MS);
    uint8_t diffMask = 0U;
    if (hasRx120) {
        for (uint8_t i = 0U; i < 4U; ++i) {
            if (shadow[i] != canStats.last120Data[i]) {
                diffMask |= static_cast<uint8_t>(1U << i);
            }
        }
    }

#if METASENSE_LEAF_120_FACTS_LOGS
    static uint32_t factSamples = 0U;
    static uint32_t factByte2HiMatches = 0U;
    static uint32_t factFullMatches = 0U;
    static uint32_t factStateSamples[kLeaf120StateClassCount] = {0U};
    static uint32_t factStateByte2HiMatches[kLeaf120StateClassCount] = {0U};
    static uint32_t factStateFullMatches[kLeaf120StateClassCount] = {0U};
    static uint32_t factRxByte2Counts[kLeaf120StateClassCount][256] = {{0U}};
    static uint32_t factRxByte2HiCounts[kLeaf120StateClassCount][16] = {{0U}};
    static uint32_t factRxDlcCounts[9] = {0U};
    static uint32_t factRxTailAnyNonZero = 0U;
    static uint32_t factRxTailAllZero = 0U;
    static uint32_t factLastLogMs = 0U;
    static bool dtPrevValid = false;
    static float dtPrevCmdNm = 0.0f;
    static uint32_t dtPrevMs = 0U;
    static uint32_t dtSamples = 0U;
    static float dtMaxAbsNmps = 0.0f;
    static float dtSumAbsNmps = 0.0f;
    static uint32_t dtSlopeClassCounts[kLeaf120SlopeClassCount] = {0U};
    static uint32_t dtSlopeClassByte2Hi[kLeaf120SlopeClassCount][16] = {{0U}};
    static uint32_t nibbleHiCounts[16] = {0U};
    static uint32_t nibbleLoCounts[16] = {0U};
    static bool nibblePrevValid = false;
    static uint8_t nibblePrevLo = 0U;
    static uint32_t nibbleLoStepOk = 0U;
    static uint32_t nibbleLoStepBad = 0U;
    static uint32_t nibbleLoStepWrap = 0U;
#if METASENSE_LEAF_120_NOISE_LOGS
    static bool noiseInitialized = false;
    static float cmdEmaNm = 0.0f;
    static float effEmaNm = 0.0f;
    static float cmdMinNm = 0.0f;
    static float cmdMaxNm = 0.0f;
    static uint32_t cmdPosCount = 0U;
    static uint32_t cmdNegCount = 0U;
    static uint32_t cmdNearZeroCount = 0U;
    static uint32_t cmdSignFlipCount = 0U;
    static int8_t cmdPrevSign = 0;
    static uint32_t effFreshCount = 0U;
    static uint32_t effStaleCount = 0U;
#endif

    if (freshRx120) {
        if (canStats.last120Len <= 8U) {
            ++factRxDlcCounts[canStats.last120Len];
        }

        bool tailAnyNonZero = false;
        if (canStats.last120Len > 4U) {
            for (uint8_t i = 4U; i < canStats.last120Len && i < 8U; ++i) {
                if (canStats.last120Data[i] != 0U) {
                    tailAnyNonZero = true;
                    break;
                }
            }
            if (tailAnyNonZero) {
                ++factRxTailAnyNonZero;
            } else {
                ++factRxTailAllZero;
            }
        }

        const uint8_t stateIdx = leaf120ShadowStateClassIndex(stateClass);
        ++factSamples;
        ++factStateSamples[stateIdx];
        ++factRxByte2Counts[stateIdx][canStats.last120Data[2]];

        const uint8_t b2 = canStats.last120Data[2];
        const uint8_t b2Hi = static_cast<uint8_t>((b2 >> 4) & 0x0FU);
        const uint8_t b2Lo = static_cast<uint8_t>(b2 & 0x0FU);
        ++factRxByte2HiCounts[stateIdx][b2Hi];
        ++nibbleHiCounts[b2Hi];
        ++nibbleLoCounts[b2Lo];
        if (nibblePrevValid) {
            const uint8_t expected = static_cast<uint8_t>((nibblePrevLo + 1U) & 0x0FU);
            if (b2Lo == expected) {
                ++nibbleLoStepOk;
                if (expected == 0U) {
                    ++nibbleLoStepWrap;
                }
            } else {
                ++nibbleLoStepBad;
            }
        }
        nibblePrevLo = b2Lo;
        nibblePrevValid = true;

        const uint8_t shadowB2Hi = static_cast<uint8_t>((shadow[2] >> 4) & 0x0FU);
        if (shadowB2Hi == b2Hi) {
            ++factByte2HiMatches;
            ++factStateByte2HiMatches[stateIdx];
        }
        if (diffMask == 0U) {
            ++factFullMatches;
            ++factStateFullMatches[stateIdx];
        }

        // Hypothesis test: does RX byte2 behavior correlate with torque slew (dTorque/dt)?
        const Leaf120CommandDecode rx120Now = decodeLeaf120Command(canStats.last120Data,
                                                                    canStats.last120Len);
#if METASENSE_LEAF_120_NOISE_LOGS
        const float cmdNm = rx120Now.torqueDemandNmBase;
        const float noiseDb = METASENSE_LEAF_120_NOISE_DB_NM;
        int8_t cmdSign = 0;
        if (cmdNm > noiseDb) {
            cmdSign = 1;
            ++cmdPosCount;
        } else if (cmdNm < -noiseDb) {
            cmdSign = -1;
            ++cmdNegCount;
        } else {
            ++cmdNearZeroCount;
        }

        if (cmdSign != 0 && cmdPrevSign != 0 && cmdSign != cmdPrevSign) {
            ++cmdSignFlipCount;
        }
        if (cmdSign != 0) {
            cmdPrevSign = cmdSign;
        }

        const LeafInvFeedback& fbNoise = MetaSense::CANBus::feedback();
        const bool effFresh = (fbNoise.torque_update_ms != 0U) &&
                              (elapsedMsSafe(nowMs, fbNoise.torque_update_ms) <= CAN_RX_TARGET_MAX_AGE_MS);
        const float effNm = effFresh ? fbNoise.torque_nm : 0.0f;
        if (effFresh) {
            ++effFreshCount;
        } else {
            ++effStaleCount;
        }

        if (!noiseInitialized) {
            noiseInitialized = true;
            cmdEmaNm = cmdNm;
            effEmaNm = effNm;
            cmdMinNm = cmdNm;
            cmdMaxNm = cmdNm;
        } else {
            const float a = constrain(METASENSE_LEAF_120_NOISE_EMA_ALPHA, 0.01f, 1.0f);
            cmdEmaNm += a * (cmdNm - cmdEmaNm);
            if (effFresh) {
                effEmaNm += a * (effNm - effEmaNm);
            }
            if (cmdNm < cmdMinNm) {
                cmdMinNm = cmdNm;
            }
            if (cmdNm > cmdMaxNm) {
                cmdMaxNm = cmdNm;
            }
        }
#endif
        if (dtPrevValid && nowMs > dtPrevMs) {
            const uint32_t dtMs = nowMs - dtPrevMs;
            const float dtSec = static_cast<float>(dtMs) / 1000.0f;
            if (dtSec > 0.0f) {
                const float dNm = rx120Now.torqueDemandNmBase - dtPrevCmdNm;
                const float nmps = dNm / dtSec;
                const float absNmps = fabsf(nmps);
                ++dtSamples;
                dtSumAbsNmps += absNmps;
                if (absNmps > dtMaxAbsNmps) {
                    dtMaxAbsNmps = absNmps;
                }

                uint8_t slopeClass = 0U; // steady
                if (absNmps <= METASENSE_LEAF_120_DT_SLOPE_STEADY_NMPS) {
                    slopeClass = 0U; // steady
                } else if (nmps > METASENSE_LEAF_120_DT_SLOPE_STEADY_NMPS) {
                    slopeClass = (absNmps >= METASENSE_LEAF_120_DT_SLOPE_HIGH_NMPS) ? 3U : 1U; // rise/high
                } else {
                    slopeClass = (absNmps >= METASENSE_LEAF_120_DT_SLOPE_HIGH_NMPS) ? 3U : 2U; // fall/high
                }

                ++dtSlopeClassCounts[slopeClass];
                ++dtSlopeClassByte2Hi[slopeClass][b2Hi];
            }
        }
        dtPrevCmdNm = rx120Now.torqueDemandNmBase;
        dtPrevMs = nowMs;
        dtPrevValid = true;
    }

    if (factLastLogMs == 0U || (nowMs - factLastLogMs) >= METASENSE_LEAF_120_FACTS_LOG_PERIOD_MS) {
        factLastLogMs = nowMs;

        uint32_t nCount = 0U;
        const uint8_t nTop = topByteFromCounts(factRxByte2Counts[0], nCount);
        uint32_t iCount = 0U;
        const uint8_t iTop = topByteFromCounts(factRxByte2Counts[1], iCount);
        uint32_t mCount = 0U;
        const uint8_t mTop = topByteFromCounts(factRxByte2Counts[2], mCount);
        uint32_t bCount = 0U;
        const uint8_t bTop = topByteFromCounts(factRxByte2Counts[3], bCount);
        uint32_t rCount = 0U;
        const uint8_t rTop = topByteFromCounts(factRxByte2Counts[4], rCount);

        const float byte2HiMatchPct = (factSamples > 0U)
            ? (100.0f * static_cast<float>(factByte2HiMatches) / static_cast<float>(factSamples))
            : 0.0f;
        const float fullMatchPct = (factSamples > 0U)
            ? (100.0f * static_cast<float>(factFullMatches) / static_cast<float>(factSamples))
            : 0.0f;
        const float meanAbsNmps = (dtSamples > 0U)
            ? (dtSumAbsNmps / static_cast<float>(dtSamples))
            : 0.0f;

        uint32_t sCount = 0U;
        const uint8_t sTop = topByteFromCounts(dtSlopeClassByte2Hi[0], sCount);
        uint32_t upCount = 0U;
        const uint8_t upTop = topByteFromCounts(dtSlopeClassByte2Hi[1], upCount);
        uint32_t dnCount = 0U;
        const uint8_t dnTop = topByteFromCounts(dtSlopeClassByte2Hi[2], dnCount);
        uint32_t hiCount = 0U;
        const uint8_t hiTop = topByteFromCounts(dtSlopeClassByte2Hi[3], hiCount);
        uint32_t b2HiTopCount = 0U;
        const uint8_t b2HiTop = topByteFromCounts(nibbleHiCounts, b2HiTopCount);
        uint32_t b2LoTopCount = 0U;
        const uint8_t b2LoTop = topByteFromCounts(nibbleLoCounts, b2LoTopCount);
        const uint32_t loStepTotal = nibbleLoStepOk + nibbleLoStepBad;
        const float loStepOkPct = (loStepTotal > 0U)
            ? (100.0f * static_cast<float>(nibbleLoStepOk) / static_cast<float>(loStepTotal))
            : 0.0f;

        Serial.printf("[VCM-120-FACTS] rule=%s samples=%lu b2hi_match=%lu(%.1f%%) full_match=%lu(%.1f%%)"
                  " | N(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu) I(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu)"
                  " M(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu) B(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu)"
                      " R(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu)"
                      " | dlc4=%lu dlc5=%lu dlc6=%lu dlc7=%lu dlc8=%lu tailNZ=%lu tailZ=%lu\n",
                      leaf120ShadowStrategyName(),
                      static_cast<unsigned long>(factSamples),
                  static_cast<unsigned long>(factByte2HiMatches),
                  byte2HiMatchPct,
                      static_cast<unsigned long>(factFullMatches),
                      fullMatchPct,
                      static_cast<unsigned long>(factStateSamples[0]),
                  static_cast<unsigned long>(factStateByte2HiMatches[0]),
                      static_cast<unsigned long>(factStateFullMatches[0]),
                      static_cast<unsigned>(nTop),
                      static_cast<unsigned long>(nCount),
                      static_cast<unsigned long>(factStateSamples[1]),
                  static_cast<unsigned long>(factStateByte2HiMatches[1]),
                      static_cast<unsigned long>(factStateFullMatches[1]),
                      static_cast<unsigned>(iTop),
                      static_cast<unsigned long>(iCount),
                      static_cast<unsigned long>(factStateSamples[2]),
                  static_cast<unsigned long>(factStateByte2HiMatches[2]),
                      static_cast<unsigned long>(factStateFullMatches[2]),
                      static_cast<unsigned>(mTop),
                      static_cast<unsigned long>(mCount),
                      static_cast<unsigned long>(factStateSamples[3]),
                  static_cast<unsigned long>(factStateByte2HiMatches[3]),
                      static_cast<unsigned long>(factStateFullMatches[3]),
                      static_cast<unsigned>(bTop),
                      static_cast<unsigned long>(bCount),
                      static_cast<unsigned long>(factStateSamples[4]),
                  static_cast<unsigned long>(factStateByte2HiMatches[4]),
                      static_cast<unsigned long>(factStateFullMatches[4]),
                      static_cast<unsigned>(rTop),
                      static_cast<unsigned long>(rCount),
                      static_cast<unsigned long>(factRxDlcCounts[4]),
                      static_cast<unsigned long>(factRxDlcCounts[5]),
                      static_cast<unsigned long>(factRxDlcCounts[6]),
                      static_cast<unsigned long>(factRxDlcCounts[7]),
                      static_cast<unsigned long>(factRxDlcCounts[8]),
                      static_cast<unsigned long>(factRxTailAnyNonZero),
                      static_cast<unsigned long>(factRxTailAllZero));
        Serial0.printf("[VCM-120-FACTS] rule=%s samples=%lu b2hi_match=%lu(%.1f%%) full_match=%lu(%.1f%%)"
                       " | N(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu) I(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu)"
                       " M(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu) B(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu)"
                       " R(n=%lu,m=%lu,f=%lu,topB2=0x%02X:%lu)"
                       " | dlc4=%lu dlc5=%lu dlc6=%lu dlc7=%lu dlc8=%lu tailNZ=%lu tailZ=%lu\n",
                       leaf120ShadowStrategyName(),
                       static_cast<unsigned long>(factSamples),
                   static_cast<unsigned long>(factByte2HiMatches),
                   byte2HiMatchPct,
                       static_cast<unsigned long>(factFullMatches),
                       fullMatchPct,
                       static_cast<unsigned long>(factStateSamples[0]),
                   static_cast<unsigned long>(factStateByte2HiMatches[0]),
                       static_cast<unsigned long>(factStateFullMatches[0]),
                       static_cast<unsigned>(nTop),
                       static_cast<unsigned long>(nCount),
                       static_cast<unsigned long>(factStateSamples[1]),
                   static_cast<unsigned long>(factStateByte2HiMatches[1]),
                       static_cast<unsigned long>(factStateFullMatches[1]),
                       static_cast<unsigned>(iTop),
                       static_cast<unsigned long>(iCount),
                       static_cast<unsigned long>(factStateSamples[2]),
                   static_cast<unsigned long>(factStateByte2HiMatches[2]),
                       static_cast<unsigned long>(factStateFullMatches[2]),
                       static_cast<unsigned>(mTop),
                       static_cast<unsigned long>(mCount),
                       static_cast<unsigned long>(factStateSamples[3]),
                   static_cast<unsigned long>(factStateByte2HiMatches[3]),
                       static_cast<unsigned long>(factStateFullMatches[3]),
                       static_cast<unsigned>(bTop),
                       static_cast<unsigned long>(bCount),
                       static_cast<unsigned long>(factStateSamples[4]),
                   static_cast<unsigned long>(factStateByte2HiMatches[4]),
                       static_cast<unsigned long>(factStateFullMatches[4]),
                       static_cast<unsigned>(rTop),
                       static_cast<unsigned long>(rCount),
                       static_cast<unsigned long>(factRxDlcCounts[4]),
                       static_cast<unsigned long>(factRxDlcCounts[5]),
                       static_cast<unsigned long>(factRxDlcCounts[6]),
                       static_cast<unsigned long>(factRxDlcCounts[7]),
                       static_cast<unsigned long>(factRxDlcCounts[8]),
                       static_cast<unsigned long>(factRxTailAnyNonZero),
                       static_cast<unsigned long>(factRxTailAllZero));

        Serial.printf("[VCM-120-DT] samples=%lu meanAbsNmps=%.1f maxAbsNmps=%.1f"
                  " | steady(n=%lu,topB2hi=0x%X:%lu) rise(n=%lu,topB2hi=0x%X:%lu)"
                  " fall(n=%lu,topB2hi=0x%X:%lu) highSlew(n=%lu,topB2hi=0x%X:%lu)\n",
                      static_cast<unsigned long>(dtSamples),
                      meanAbsNmps,
                      dtMaxAbsNmps,
                      static_cast<unsigned long>(dtSlopeClassCounts[0]),
                      static_cast<unsigned>(sTop),
                      static_cast<unsigned long>(sCount),
                      static_cast<unsigned long>(dtSlopeClassCounts[1]),
                      static_cast<unsigned>(upTop),
                      static_cast<unsigned long>(upCount),
                      static_cast<unsigned long>(dtSlopeClassCounts[2]),
                      static_cast<unsigned>(dnTop),
                      static_cast<unsigned long>(dnCount),
                      static_cast<unsigned long>(dtSlopeClassCounts[3]),
                      static_cast<unsigned>(hiTop),
                      static_cast<unsigned long>(hiCount));
        Serial0.printf("[VCM-120-DT] samples=%lu meanAbsNmps=%.1f maxAbsNmps=%.1f"
                       " | steady(n=%lu,topB2hi=0x%X:%lu) rise(n=%lu,topB2hi=0x%X:%lu)"
                       " fall(n=%lu,topB2hi=0x%X:%lu) highSlew(n=%lu,topB2hi=0x%X:%lu)\n",
                       static_cast<unsigned long>(dtSamples),
                       meanAbsNmps,
                       dtMaxAbsNmps,
                       static_cast<unsigned long>(dtSlopeClassCounts[0]),
                       static_cast<unsigned>(sTop),
                       static_cast<unsigned long>(sCount),
                       static_cast<unsigned long>(dtSlopeClassCounts[1]),
                       static_cast<unsigned>(upTop),
                       static_cast<unsigned long>(upCount),
                       static_cast<unsigned long>(dtSlopeClassCounts[2]),
                       static_cast<unsigned>(dnTop),
                       static_cast<unsigned long>(dnCount),
                       static_cast<unsigned long>(dtSlopeClassCounts[3]),
                       static_cast<unsigned>(hiTop),
                       static_cast<unsigned long>(hiCount));
        Serial.printf("[VCM-120-NIBBLE] b2_hi_top=0x%X:%lu b2_lo_top=0x%X:%lu lo_step_ok=%lu/%lu(%.1f%%) wraps=%lu\n",
                      static_cast<unsigned>(b2HiTop),
                      static_cast<unsigned long>(b2HiTopCount),
                      static_cast<unsigned>(b2LoTop),
                      static_cast<unsigned long>(b2LoTopCount),
                      static_cast<unsigned long>(nibbleLoStepOk),
                      static_cast<unsigned long>(loStepTotal),
                      loStepOkPct,
                      static_cast<unsigned long>(nibbleLoStepWrap));
        Serial0.printf("[VCM-120-NIBBLE] b2_hi_top=0x%X:%lu b2_lo_top=0x%X:%lu lo_step_ok=%lu/%lu(%.1f%%) wraps=%lu\n",
                       static_cast<unsigned>(b2HiTop),
                       static_cast<unsigned long>(b2HiTopCount),
                       static_cast<unsigned>(b2LoTop),
                       static_cast<unsigned long>(b2LoTopCount),
                       static_cast<unsigned long>(nibbleLoStepOk),
                       static_cast<unsigned long>(loStepTotal),
                       loStepOkPct,
                       static_cast<unsigned long>(nibbleLoStepWrap));
#if METASENSE_LEAF_120_NOISE_LOGS
        const uint32_t effTotal = effFreshCount + effStaleCount;
        const float effFreshPct = (effTotal > 0U)
            ? (100.0f * static_cast<float>(effFreshCount) / static_cast<float>(effTotal))
            : 0.0f;
        Serial.printf("[VCM-120-NOISE] cmd(min=%.2f max=%.2f ema=%.2f pos=%lu neg=%lu zero=%lu flips=%lu db=%.2f)"
                      " eff(ema=%.2f fresh=%lu/%lu %.1f%%)\n",
                      cmdMinNm,
                      cmdMaxNm,
                      cmdEmaNm,
                      static_cast<unsigned long>(cmdPosCount),
                      static_cast<unsigned long>(cmdNegCount),
                      static_cast<unsigned long>(cmdNearZeroCount),
                      static_cast<unsigned long>(cmdSignFlipCount),
                      METASENSE_LEAF_120_NOISE_DB_NM,
                      effEmaNm,
                      static_cast<unsigned long>(effFreshCount),
                      static_cast<unsigned long>(effTotal),
                      effFreshPct);
        Serial0.printf("[VCM-120-NOISE] cmd(min=%.2f max=%.2f ema=%.2f pos=%lu neg=%lu zero=%lu flips=%lu db=%.2f)"
                       " eff(ema=%.2f fresh=%lu/%lu %.1f%%)\n",
                       cmdMinNm,
                       cmdMaxNm,
                       cmdEmaNm,
                       static_cast<unsigned long>(cmdPosCount),
                       static_cast<unsigned long>(cmdNegCount),
                       static_cast<unsigned long>(cmdNearZeroCount),
                       static_cast<unsigned long>(cmdSignFlipCount),
                       METASENSE_LEAF_120_NOISE_DB_NM,
                       effEmaNm,
                       static_cast<unsigned long>(effFreshCount),
                       static_cast<unsigned long>(effTotal),
                       effFreshPct);
#endif
    }
#endif

    Serial.printf("[VCM-120-SHADOW] mode=%s rule=%s sent=%d bits(R=%d H=%d B=%d G=%d) tq=%.2f rpm=%.1f tx=%02X %02X %02X %02X rx=%02X %02X %02X %02X age=%lums diff=0x%X\n",
                  leaf120ShadowStateClassName(stateClass),
                  leaf120ShadowStrategyName(),
                  txSent ? 1 : 0,
                  readyBit ? 1 : 0,
                  hvOkBit ? 1 : 0,
                  resolvedBrakeBit ? 1 : 0,
                  resolvedGearDriveBit ? 1 : 0,
                  torqueDemandNm,
                  rpmForState,
                  static_cast<unsigned>(shadow[0]),
                  static_cast<unsigned>(shadow[1]),
                  static_cast<unsigned>(shadow[2]),
                  static_cast<unsigned>(shadow[3]),
                  static_cast<unsigned>(hasRx120 ? canStats.last120Data[0] : 0U),
                  static_cast<unsigned>(hasRx120 ? canStats.last120Data[1] : 0U),
                  static_cast<unsigned>(hasRx120 ? canStats.last120Data[2] : 0U),
                  static_cast<unsigned>(hasRx120 ? canStats.last120Data[3] : 0U),
                  static_cast<unsigned long>(hasRx120 ? rxAgeMs : 0U),
                  static_cast<unsigned>(diffMask));
    Serial0.printf("[VCM-120-SHADOW] mode=%s rule=%s sent=%d bits(R=%d H=%d B=%d G=%d) tq=%.2f rpm=%.1f tx=%02X %02X %02X %02X rx=%02X %02X %02X %02X age=%lums diff=0x%X\n",
                   leaf120ShadowStateClassName(stateClass),
                   leaf120ShadowStrategyName(),
                   txSent ? 1 : 0,
                   readyBit ? 1 : 0,
                   hvOkBit ? 1 : 0,
                   resolvedBrakeBit ? 1 : 0,
                   resolvedGearDriveBit ? 1 : 0,
                   torqueDemandNm,
                   rpmForState,
                   static_cast<unsigned>(shadow[0]),
                   static_cast<unsigned>(shadow[1]),
                   static_cast<unsigned>(shadow[2]),
                   static_cast<unsigned>(shadow[3]),
                   static_cast<unsigned>(hasRx120 ? canStats.last120Data[0] : 0U),
                   static_cast<unsigned>(hasRx120 ? canStats.last120Data[1] : 0U),
                   static_cast<unsigned>(hasRx120 ? canStats.last120Data[2] : 0U),
                   static_cast<unsigned>(hasRx120 ? canStats.last120Data[3] : 0U),
                   static_cast<unsigned long>(hasRx120 ? rxAgeMs : 0U),
                   static_cast<unsigned>(diffMask));
#else
    (void)nowMs;
    (void)torqueDemandNm;
    (void)readyBit;
    (void)hvOkBit;
    (void)brakeBit;
    (void)gearDriveBit;
    (void)txSent;
#endif
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
    memset(s_leafLast120ShadowData, 0, sizeof(s_leafLast120ShadowData));
    s_leafLast120ShadowMs = 0;
    s_leaf120PayloadTorqueNm = 0.0f;
    s_leaf120PayloadTorqueRaw = 0;
    s_leaf120PayloadMs = 0;
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
    memset(leaf120B2PosCounts, 0, sizeof(leaf120B2PosCounts));
    memset(leaf120B2NegCounts, 0, sizeof(leaf120B2NegCounts));
    memset(leaf120B2ZeroCounts, 0, sizeof(leaf120B2ZeroCounts));
    memset(leaf120B2BrakeCounts, 0, sizeof(leaf120B2BrakeCounts));
    leaf120B2PosSamples = 0;
    leaf120B2NegSamples = 0;
    leaf120B2ZeroSamples = 0;
    leaf120B2BrakeSamples = 0;
    memset(leaf120CrcCandidateMatches, 0, sizeof(leaf120CrcCandidateMatches));
    leaf120CrcSamples = 0;
    memset(leaf120CrcBrakeSamples, 0, sizeof(leaf120CrcBrakeSamples));
    memset(leaf120Ref00StateCorrectedMatchesByBrake, 0, sizeof(leaf120Ref00StateCorrectedMatchesByBrake));
    memset(leaf120Ref00Fixed8MatchesByBrake, 0, sizeof(leaf120Ref00Fixed8MatchesByBrake));
    memset(leaf120CrcRefIdLoMatchesByBrake, 0, sizeof(leaf120CrcRefIdLoMatchesByBrake));
    memset(leaf120CrcCmdStateSamples, 0, sizeof(leaf120CrcCmdStateSamples));
    memset(leaf120Ref00StateCorrectedMatchesByCmdState, 0, sizeof(leaf120Ref00StateCorrectedMatchesByCmdState));
    memset(leaf120Ref00Fixed8MatchesByCmdState, 0, sizeof(leaf120Ref00Fixed8MatchesByCmdState));
    memset(leaf120CrcRefIdLoMatchesByCmdState, 0, sizeof(leaf120CrcRefIdLoMatchesByCmdState));
    memset(leaf120CrcMotorHiNibSamples, 0, sizeof(leaf120CrcMotorHiNibSamples));
    memset(leaf120Ref00StateCorrectedMatchesByMotorHiNib, 0, sizeof(leaf120Ref00StateCorrectedMatchesByMotorHiNib));
    memset(leaf120Ref00Fixed8MatchesByMotorHiNib, 0, sizeof(leaf120Ref00Fixed8MatchesByMotorHiNib));
    memset(leaf120CrcRefIdLoMatchesByMotorHiNib, 0, sizeof(leaf120CrcRefIdLoMatchesByMotorHiNib));
    memset(leaf120MotorScopeSamples, 0, sizeof(leaf120MotorScopeSamples));
    memset(leaf120MotorScopeDirectMatches, 0, sizeof(leaf120MotorScopeDirectMatches));
    memset(leaf120MotorScopeStateFixMatches, 0, sizeof(leaf120MotorScopeStateFixMatches));
    memset(leaf120MotorScopeResidueByState, 0, sizeof(leaf120MotorScopeResidueByState));
    memset(leaf120MotorScopeTopResidueByState, 0, sizeof(leaf120MotorScopeTopResidueByState));
    memset(leaf120MotorScopeTopResidueCountByState, 0, sizeof(leaf120MotorScopeTopResidueCountByState));
    memset(leaf120CrcNibbleSamples, 0, sizeof(leaf120CrcNibbleSamples));
    memset(leaf120Ref00StateCorrectedMatchesByNibble, 0, sizeof(leaf120Ref00StateCorrectedMatchesByNibble));
    memset(leaf120Ref00Fixed8MatchesByNibble, 0, sizeof(leaf120Ref00Fixed8MatchesByNibble));
    memset(leaf120CrcRefIdLoMatchesByNibble, 0, sizeof(leaf120CrcRefIdLoMatchesByNibble));
    leaf120TxExpSamples = 0U;
    leaf120TxExpExactCrcMatches = 0U;
    leaf120TxExpHiCtrCrcMatches = 0U;
    leaf120TxExpLoCtrCrcMatches = 0U;
    leaf120TxExpHiCtrB2Matches = 0U;
    leaf120TxExpLoCtrB2Matches = 0U;
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

    // --- START request switch (active HIGH, debounced) ---
    {
        const bool startReqNow = (digitalRead(MetaSense::Globals::kStartRequestPin) == HIGH);
        if (startReqNow != prevStartRequestState) {
            if ((now - startRequestDebounceMs) >= kStartRequestDebounceThresholdMs) {
                startRequestDebounceMs = now;
                prevStartRequestState = startReqNow;
                if (startReqNow) {
                    if (isStartControlWindow()) {
                        MetaSense::HardwareOutputStateMachine::requestStartReset();
                        Serial.println("[START-BTN] start/reset request latched");
                        Serial0.println("[START-BTN] start/reset request latched");
                    } else {
                        Serial.println("[START-BTN] press ignored outside INIT/START window");
                        Serial0.println("[START-BTN] press ignored outside INIT/START window");
                    }
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
    const bool canRpmFrameFresh = (lastCanRpmFrameMs != 0U) &&
                                  (elapsedMsSafe(now, lastCanRpmFrameMs) <= CAN_RX_TARGET_MAX_AGE_MS);
    bool canValid = canRpmAllowed && MetaSense::CANBus::isReady() && canRpmFrameFresh;
    float rpmRaw = 0.0f;
    canFallbackActive = canRpmAllowed && !canValid;
    activeRpmFromCan = canValid;
    const float emotorRpmRaw = canRpmFrameFresh ? leafCanRpmMonitor : canRpm;
    const float rpmRatio = (MetaSense::Settings::virtGearRatio > 0.01f)
        ? MetaSense::Settings::virtGearRatio
        : 1.45f;
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
        // Keep UI demand visible even if 0x120 TX path is stale/offline.
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
    const bool relayInverterFault = tele.leaf_invFault ||
                                    tele.leaf_invLimp ||
                                    (leafFbNativeStatus.mg_error_codes != 0U);
    const bool canTelemetryReadyForStart = MetaSense::CANBus::isReady() &&
                                           canStatusFresh &&
                                           canHvFresh;
    const bool sensorsReadyForStart = isfinite(tele.rpm) &&
                                      isfinite(tele.loadKg) &&
                                      isfinite(tele.lambdaValue);

    MetaSense::HardwareOutputStateMachine::update(
        engineThrottlePercent,
        tele.rpmTarget,
        tele.rpm,
        primaryBrakeSignedPercent,
        tele.vcuHvVoltage,
        relayInverterStatusReady,
        relayInverterReady,
        relayInverterFault,
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
    if (canStatsDiag.rx120Frames != s_corrLast120Frames && canStatsDiag.last120Len >= 2U) {
        const float torqueNm = leafFbDiag.torque_nm;
        const Leaf120CommandDecode id120FrameDecoded = decodeLeaf120Command(canStatsDiag.last120Data,
                                                                            canStatsDiag.last120Len);
        const uint8_t id120FrameB2 = id120FrameDecoded.unknown120_2;
        const int16_t id120FrameCmdRaw = id120FrameDecoded.torqueDemandSignedBe;
        constexpr int16_t kLeaf120DirectionThresholdRaw = 64;
        if (id120FrameCmdRaw > kLeaf120DirectionThresholdRaw) {
            ++leaf120B2PosSamples;
            ++leaf120B2PosCounts[id120FrameB2];
        } else if (id120FrameCmdRaw < -kLeaf120DirectionThresholdRaw) {
            ++leaf120B2NegSamples;
            ++leaf120B2NegCounts[id120FrameB2];
        } else {
            ++leaf120B2ZeroSamples;
            ++leaf120B2ZeroCounts[id120FrameB2];
        }
        if (fabsf(tele.brakeTorqueNm) > 1.0f) {
            ++leaf120B2BrakeSamples;
            ++leaf120B2BrakeCounts[id120FrameB2];
        }
        const uint32_t crcCandidateMask = computeLeaf120CrcCandidateMask(canStatsDiag.last120Data,
                                          canStatsDiag.last120Len);
        const bool brakeActive120 = fabsf(tele.brakeTorqueNm) > 1.0f;
        const uint8_t brakeBucket120 = brakeActive120 ? 1U : 0U;
        constexpr uint8_t kLeaf120CmdStateZero = 0U;
        constexpr uint8_t kLeaf120CmdStateMotor = 1U;
        constexpr uint8_t kLeaf120CmdStateRegen = 2U;
        uint8_t cmdState120 = kLeaf120CmdStateZero;
        if (id120FrameCmdRaw > kLeaf120DirectionThresholdRaw) {
            cmdState120 = kLeaf120CmdStateMotor;
        } else if (id120FrameCmdRaw < -kLeaf120DirectionThresholdRaw) {
            cmdState120 = kLeaf120CmdStateRegen;
        }
        const uint8_t modeNibble120 = static_cast<uint8_t>((canStatsDiag.last120Data[2] >> 4U) & 0x0FU);
        const bool cmdStateIsMotor = (cmdState120 == kLeaf120CmdStateMotor);
        ++leaf120CrcSamples;
        ++leaf120CrcBrakeSamples[brakeBucket120];
        ++leaf120CrcCmdStateSamples[cmdState120];
        if (cmdStateIsMotor) {
            ++leaf120CrcMotorHiNibSamples[modeNibble120];
        }
        ++leaf120CrcNibbleSamples[modeNibble120];
        for (uint8_t bit = 0U; bit < kLeaf120CrcCandidateCount; ++bit) {
            if ((crcCandidateMask & static_cast<uint16_t>(1U << bit)) != 0U) {
                ++leaf120CrcCandidateMatches[bit];
            }
        }
        if ((crcCandidateMask & static_cast<uint16_t>(1U << 15U)) != 0U) {
            ++leaf120CrcRefIdLoMatchesByBrake[brakeBucket120];
            ++leaf120CrcRefIdLoMatchesByCmdState[cmdState120];
            if (cmdStateIsMotor) {
                ++leaf120CrcRefIdLoMatchesByMotorHiNib[modeNibble120];
            }
            ++leaf120CrcRefIdLoMatchesByNibble[modeNibble120];
        }
        const uint8_t crc120Rx = canStatsDiag.last120Data[3];
        const uint8_t crc120Ref00 = crc8Lsb(canStatsDiag.last120Data, 3U, 0xB8U, 0x00U, 0x00U);
        const uint8_t crc120State = static_cast<uint8_t>(canStatsDiag.last120Data[2] & 0x0FU);

        if (cmdStateIsMotor) {
            const uint8_t crcScopeBase0 = crc120Ref00;
            const uint8_t scopeIdLoPayload[4] = {
                0x20U,
                canStatsDiag.last120Data[0],
                canStatsDiag.last120Data[1],
                canStatsDiag.last120Data[2]
            };
            const uint8_t scopeIdBePayload[5] = {
                0x01U,
                0x20U,
                canStatsDiag.last120Data[0],
                canStatsDiag.last120Data[1],
                canStatsDiag.last120Data[2]
            };
            const uint8_t crcScopeBase1 = crc8Lsb(scopeIdLoPayload, 4U, 0xB8U, 0x00U, 0x00U);
            const uint8_t crcScopeBase2 = crc8Lsb(scopeIdBePayload, 5U, 0xB8U, 0x00U, 0x00U);
            const uint8_t crcScopeBase3 = crc8Msb(scopeIdLoPayload, 4U, 0x1DU, 0xFFU, 0xFFU);
            const uint8_t scopeBases[kLeaf120MotorScopeCount] = {
                crcScopeBase0,
                crcScopeBase1,
                crcScopeBase2,
                crcScopeBase3
            };

            for (uint8_t scopeIdx = 0U; scopeIdx < kLeaf120MotorScopeCount; ++scopeIdx) {
                ++leaf120MotorScopeSamples[scopeIdx];
                const uint8_t scopeBase = scopeBases[scopeIdx];
                if (scopeBase == crc120Rx) {
                    ++leaf120MotorScopeDirectMatches[scopeIdx];
                }

                const uint8_t scopeResidue = static_cast<uint8_t>(crc120Rx ^ scopeBase);
                const uint32_t scopeResidueCount = ++leaf120MotorScopeResidueByState[scopeIdx][crc120State][scopeResidue];
                if (scopeResidueCount > leaf120MotorScopeTopResidueCountByState[scopeIdx][crc120State]) {
                    leaf120MotorScopeTopResidueCountByState[scopeIdx][crc120State] = scopeResidueCount;
                    leaf120MotorScopeTopResidueByState[scopeIdx][crc120State] = scopeResidue;
                }

                const uint8_t scopeCorrected = static_cast<uint8_t>(scopeBase ^ leaf120MotorScopeTopResidueByState[scopeIdx][crc120State]);
                if (scopeCorrected == crc120Rx) {
                    ++leaf120MotorScopeStateFixMatches[scopeIdx];
                }
            }
        }

        const uint8_t crc120Residue = static_cast<uint8_t>(crc120Rx ^ crc120Ref00);
        const uint32_t crc120ResidueCount = ++leaf120Ref00ResidueByState[crc120State][crc120Residue];
        if (crc120ResidueCount > leaf120Ref00TopResidueCountByState[crc120State]) {
            leaf120Ref00TopResidueCountByState[crc120State] = crc120ResidueCount;
            leaf120Ref00TopResidueByState[crc120State] = crc120Residue;
        }
        const uint8_t crc120Corrected = static_cast<uint8_t>(crc120Ref00 ^ leaf120Ref00TopResidueByState[crc120State]);
        if (crc120Corrected == crc120Rx) {
            ++leaf120Ref00StateCorrectedMatches;
            ++leaf120Ref00StateCorrectedMatchesByBrake[brakeBucket120];
            ++leaf120Ref00StateCorrectedMatchesByCmdState[cmdState120];
            if (cmdStateIsMotor) {
                ++leaf120Ref00StateCorrectedMatchesByMotorHiNib[modeNibble120];
            }
            ++leaf120Ref00StateCorrectedMatchesByNibble[modeNibble120];
        }
        const uint8_t crc120Fixed8 = static_cast<uint8_t>(
            crc120Ref00 ^ kLeaf120Ref00ResidueByState8[crc120State & 0x07U]);
        if (crc120Fixed8 == crc120Rx) {
            ++leaf120Ref00Fixed8Matches;
            ++leaf120Ref00Fixed8MatchesByBrake[brakeBucket120];
            ++leaf120Ref00Fixed8MatchesByCmdState[cmdState120];
            if (cmdStateIsMotor) {
                ++leaf120Ref00Fixed8MatchesByMotorHiNib[modeNibble120];
            }
            ++leaf120Ref00Fixed8MatchesByNibble[modeNibble120];
        }

        // TX experiment gate: use captured 0x120 torque + one nibble copy, regenerate CRC, and compare.
        {
            ++leaf120TxExpSamples;

            const uint8_t crcExact = static_cast<uint8_t>(
                crc8Lsb120(canStatsDiag.last120Data, 3U) ^
                kLeaf120Ref00ResidueByState8[canStatsDiag.last120Data[2] & 0x07U]);
            if (crcExact == crc120Rx) {
                ++leaf120TxExpExactCrcMatches;
            }

            uint8_t modelFrame[4] = {0U, 0U, 0U, 0U};
            const bool expReadyBit = tele.vcuReady;
            const bool expHvOkBit = tele.vcuHvVoltage >= 250.0f;
            const bool expBrakeBit = fabsf(tele.brakeTorqueNm) > 1.0f;
            const bool expGearDriveBit = (id120FrameDecoded.torqueDemandSignedBe >= 0);
            buildLeaf120ShadowFrame(id120FrameDecoded.torqueDemandNmBase,
                                    expReadyBit,
                                    expHvOkBit,
                                    expBrakeBit,
                                    expGearDriveBit,
                                    modelFrame);

            const uint8_t rxB2 = canStatsDiag.last120Data[2];

            // Hypothesis A: high nibble is rolling counter from capture, low nibble from our model.
            uint8_t synthHiCtr[4] = {
                canStatsDiag.last120Data[0],
                canStatsDiag.last120Data[1],
                static_cast<uint8_t>((rxB2 & 0xF0U) | (modelFrame[2] & 0x0FU)),
                0U
            };
            synthHiCtr[3] = static_cast<uint8_t>(
                crc8Lsb120(synthHiCtr, 3U) ^ kLeaf120Ref00ResidueByState8[synthHiCtr[2] & 0x07U]);
            if (synthHiCtr[3] == crc120Rx) {
                ++leaf120TxExpHiCtrCrcMatches;
            }
            if (synthHiCtr[2] == rxB2) {
                ++leaf120TxExpHiCtrB2Matches;
            }

            // Hypothesis B: low nibble is rolling counter from capture, high nibble from our model.
            uint8_t synthLoCtr[4] = {
                canStatsDiag.last120Data[0],
                canStatsDiag.last120Data[1],
                static_cast<uint8_t>((modelFrame[2] & 0xF0U) | (rxB2 & 0x0FU)),
                0U
            };
            synthLoCtr[3] = static_cast<uint8_t>(
                crc8Lsb120(synthLoCtr, 3U) ^ kLeaf120Ref00ResidueByState8[synthLoCtr[2] & 0x07U]);
            if (synthLoCtr[3] == crc120Rx) {
                ++leaf120TxExpLoCtrCrcMatches;
            }
            if (synthLoCtr[2] == rxB2) {
                ++leaf120TxExpLoCtrB2Matches;
            }
        }

        if (!s_corrTorqueEmaInit) {
            s_corrTorqueEmaNm = torqueNm;
            s_corrTorqueEmaInit = true;
        } else {
            const float kCorrTorqueEmaAlpha = 0.20f;
            s_corrTorqueEmaNm += kCorrTorqueEmaAlpha * (torqueNm - s_corrTorqueEmaNm);
        }
        const uint16_t be01 = (static_cast<uint16_t>(canStatsDiag.last120Data[0]) << 8) |
                              static_cast<uint16_t>(canStatsDiag.last120Data[1]);
        const uint16_t le01 = static_cast<uint16_t>(canStatsDiag.last120Data[0]) |
                              (static_cast<uint16_t>(canStatsDiag.last120Data[1]) << 8);
        const int16_t be01s = static_cast<int16_t>(be01);
        const bool strictZeroHold = (fabsf(torqueNm) <= 0.10f) &&
                                    (fabsf(leafFbDiag.rpm) <= 100.0f) &&
                                    (abs(static_cast<int>(be01s)) <= 64);
        s_corrFitFrozenNow = strictZeroHold;
        if (strictZeroHold) {
            s_corrFitZeroHoldSkipN += 1U;
        } else {
            corrUpdate(s_corr120Be01Torque,
                       static_cast<double>(be01s),
                       static_cast<double>(s_corrTorqueEmaNm));
            corrUpdate(s_corr120Le01Torque,
                       static_cast<double>(static_cast<int16_t>(le01)),
                       static_cast<double>(s_corrTorqueEmaNm));
            s_corrFitAcceptedN += 1U;
        }
        s_corrLast120Frames = canStatsDiag.rx120Frames;
    }

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
        uint8_t payload1daWithIdLo[8] = {0U};
        payload1daWithIdLo[0] = static_cast<uint8_t>(0x01DAU & 0xFFU);
        memcpy(&payload1daWithIdLo[1], leafFbDiag.id1da_raw, 7U);
        const uint8_t crc1daIdLo = crc8Msb(payload1daWithIdLo, 8U, 0x1DU, 0xFFU, 0xFFU);
        const uint8_t crc1daIdLoResidue = static_cast<uint8_t>(crc1daRx ^ crc1daIdLo);
        const uint32_t idLoResidueCount = ++leaf1daIdLoResidueByClock[crc1daClock][crc1daIdLoResidue];
        if (idLoResidueCount > leaf1daIdLoTopResidueCountByClock[crc1daClock]) {
            leaf1daIdLoTopResidueCountByClock[crc1daClock] = idLoResidueCount;
            leaf1daIdLoTopResidueByClock[crc1daClock] = crc1daIdLoResidue;
        }
        const uint8_t crc1daClockCorrected = static_cast<uint8_t>(crc1daIdLo ^ leaf1daIdLoTopResidueByClock[crc1daClock]);
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

    if (s_leafPreStatusLastMs == 0U || (now - s_leafPreStatusLastMs) >= CAN_PRE_DIAG_LOG_PERIOD_MS) {
        const unsigned long feedbackAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, lastCanLeafAnyUpdate));
        const unsigned long id1daAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.rpm_update_ms));
        const unsigned long id1dbAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.torque_update_ms));
        const unsigned long id1dcAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.temps_update_ms));
        const unsigned long raw55bAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last55bMs));
        const unsigned long raw11aAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last11aMs));
        const unsigned long raw120AgeMs = static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.last120Ms));
        const unsigned long unkAgeMs = static_cast<unsigned long>(elapsedMsSafe(now, canStatsDiag.lastUnknownMs));
        const unsigned long legacy55xCompat120Count = static_cast<unsigned long>(canStatsDiag.rx120Frames);
        const unsigned long legacy55xCompat120AgeMs = raw120AgeMs;
        const Leaf120CommandDecode id120Decoded = decodeLeaf120Command(canStatsDiag.last120Data,
                                        canStatsDiag.last120Len);
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
#if METASENSE_LEAF_120_NOISE_LOGS
        static bool s_vcmNoiseInit = false;
        static bool s_vcmAdjNoiseInit = false;
        static float s_vcmCmdRawMinNm = 0.0f;
        static float s_vcmCmdRawMaxNm = 0.0f;
        static float s_vcmCmdRawEmaNm = 0.0f;
        static float s_vcmCmdAdjMinNm = 0.0f;
        static float s_vcmCmdAdjMaxNm = 0.0f;
        static float s_vcmCmdAdjEmaNm = 0.0f;
        static float s_vcmEffEmaNm = 0.0f;
        static uint32_t s_vcmCmdPosCount = 0U;
        static uint32_t s_vcmCmdNegCount = 0U;
        static uint32_t s_vcmCmdZeroCount = 0U;
        static uint32_t s_vcmCmdFlipCount = 0U;
        static uint32_t s_vcmCmdOutlierCount = 0U;
        static int8_t s_vcmPrevSign = 0;
        static uint32_t s_vcmTqFreshCount = 0U;
        static uint32_t s_vcmTqStaleCount = 0U;
        static uint32_t s_vcm120FreshCount = 0U;
        static uint32_t s_vcm120StaleCount = 0U;

        const float noiseDbNm = METASENSE_LEAF_120_NOISE_DB_NM;
        const float noiseAlpha = constrain(METASENSE_LEAF_120_NOISE_EMA_ALPHA, 0.01f, 1.0f);
        const float noiseOutlierAbsNm = METASENSE_LEAF_120_NOISE_OUTLIER_ABS_NM;
        const float cmdRawForNoiseNm = id120CmdEstNmNow;
        const float cmdAdjForNoiseNm = id120CmdAdjNmNow;
        const float effForNoiseNm = tqEffNmNow;
        const bool tqFreshNow = (id1dbAgeMs <= CAN_RX_TARGET_MAX_AGE_MS);
        const bool cmd120FreshNow = (raw120AgeMs <= CAN_RX_TARGET_MAX_AGE_MS);
        const bool cmdAdjOutlier = fabsf(cmdAdjForNoiseNm) > noiseOutlierAbsNm;

        if (cmdAdjOutlier) {
            ++s_vcmCmdOutlierCount;
        }

        int8_t cmdSignNow = 0;
        if (!cmdAdjOutlier) {
            if (cmdAdjForNoiseNm > noiseDbNm) {
                cmdSignNow = 1;
                ++s_vcmCmdPosCount;
            } else if (cmdAdjForNoiseNm < -noiseDbNm) {
                cmdSignNow = -1;
                ++s_vcmCmdNegCount;
            } else {
                ++s_vcmCmdZeroCount;
            }

            if (cmdSignNow != 0 && s_vcmPrevSign != 0 && cmdSignNow != s_vcmPrevSign) {
                ++s_vcmCmdFlipCount;
            }
            if (cmdSignNow != 0) {
                s_vcmPrevSign = cmdSignNow;
            }
        } else {
            cmdSignNow = 0;
        }

        if (tqFreshNow) {
            ++s_vcmTqFreshCount;
        } else {
            ++s_vcmTqStaleCount;
        }
        if (cmd120FreshNow) {
            ++s_vcm120FreshCount;
        } else {
            ++s_vcm120StaleCount;
        }

        if (!s_vcmNoiseInit) {
            s_vcmNoiseInit = true;
            s_vcmCmdRawMinNm = cmdRawForNoiseNm;
            s_vcmCmdRawMaxNm = cmdRawForNoiseNm;
            s_vcmCmdRawEmaNm = cmdRawForNoiseNm;
            s_vcmEffEmaNm = effForNoiseNm;
        } else {
            if (cmdRawForNoiseNm < s_vcmCmdRawMinNm) {
                s_vcmCmdRawMinNm = cmdRawForNoiseNm;
            }
            if (cmdRawForNoiseNm > s_vcmCmdRawMaxNm) {
                s_vcmCmdRawMaxNm = cmdRawForNoiseNm;
            }
            s_vcmCmdRawEmaNm += noiseAlpha * (cmdRawForNoiseNm - s_vcmCmdRawEmaNm);
            s_vcmEffEmaNm += noiseAlpha * (effForNoiseNm - s_vcmEffEmaNm);
        }

        if (!cmdAdjOutlier) {
            if (!s_vcmAdjNoiseInit) {
                s_vcmAdjNoiseInit = true;
                s_vcmCmdAdjMinNm = cmdAdjForNoiseNm;
                s_vcmCmdAdjMaxNm = cmdAdjForNoiseNm;
                s_vcmCmdAdjEmaNm = cmdAdjForNoiseNm;
            } else {
                if (cmdAdjForNoiseNm < s_vcmCmdAdjMinNm) {
                    s_vcmCmdAdjMinNm = cmdAdjForNoiseNm;
                }
                if (cmdAdjForNoiseNm > s_vcmCmdAdjMaxNm) {
                    s_vcmCmdAdjMaxNm = cmdAdjForNoiseNm;
                }
                s_vcmCmdAdjEmaNm += noiseAlpha * (cmdAdjForNoiseNm - s_vcmCmdAdjEmaNm);
            }
        }
#endif
        if (lastCanAltDiagMs == 0U || (now - lastCanAltDiagMs) >= CAN_ALT_DIAG_LOG_PERIOD_MS) {
            char posTop[64];
            char negTop[64];
            char zeroTop[64];
            char brakeTop[64];
            formatLeaf120TopValues(leaf120B2PosCounts, posTop, sizeof(posTop));
            formatLeaf120TopValues(leaf120B2NegCounts, negTop, sizeof(negTop));
            formatLeaf120TopValues(leaf120B2ZeroCounts, zeroTop, sizeof(zeroTop));
            formatLeaf120TopValues(leaf120B2BrakeCounts, brakeTop, sizeof(brakeTop));

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
        #if METASENSE_LEAF_CRC_DEEP_LOGS
            Serial.printf("[VCM-120-CRC-STATS] frames=%lu match(xor=%lu,sum=%lu,inv=%lu,c07_00=%lu,c07_FF=%lu,c1D_FF=%lu,c2F_FF=%lu,c1D_00=%lu,c1D_FF_00=%lu,c1D_id120be=%lu,c1D_id120lo=%lu,c1D_ref_FF=%lu,c1D_ref_FF_00=%lu,c1D_ref_00=%lu,c1D_ref_id120be=%lu,c1D_ref_id120lo=%lu,c2F_ref_FF=%lu)\n",
                          static_cast<unsigned long>(leaf120CrcSamples),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[0]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[1]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[2]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[3]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[4]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[5]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[6]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[7]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[8]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[9]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[10]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[11]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[12]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[13]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[14]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[15]),
                          static_cast<unsigned long>(leaf120CrcCandidateMatches[16]));
            Serial0.printf("[VCM-120-CRC-STATS] frames=%lu match(xor=%lu,sum=%lu,inv=%lu,c07_00=%lu,c07_FF=%lu,c1D_FF=%lu,c2F_FF=%lu,c1D_00=%lu,c1D_FF_00=%lu,c1D_id120be=%lu,c1D_id120lo=%lu,c1D_ref_FF=%lu,c1D_ref_FF_00=%lu,c1D_ref_00=%lu,c1D_ref_id120be=%lu,c1D_ref_id120lo=%lu,c2F_ref_FF=%lu)\n",
                           static_cast<unsigned long>(leaf120CrcSamples),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[0]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[1]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[2]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[3]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[4]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[5]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[6]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[7]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[8]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[9]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[10]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[11]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[12]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[13]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[14]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[15]),
                           static_cast<unsigned long>(leaf120CrcCandidateMatches[16]));
        #endif
            const float crc120StateFixPctAll = (leaf120CrcSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatches) / static_cast<float>(leaf120CrcSamples))
                : 0.0f;
            const float crc120Fixed8PctAll = (leaf120CrcSamples > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8Matches) / static_cast<float>(leaf120CrcSamples))
                : 0.0f;
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
        #if METASENSE_LEAF_CRC_DEEP_LOGS
            Serial.printf("[VCM-120-CRC2-STATS] ref00_state_fix(match=%lu/%lu fixed8=%lu/%lu s0=0x%02X:%lu s1=0x%02X:%lu s2=0x%02X:%lu s3=0x%02X:%lu s4=0x%02X:%lu s5=0x%02X:%lu s6=0x%02X:%lu s7=0x%02X:%lu s8=0x%02X:%lu s9=0x%02X:%lu sA=0x%02X:%lu sB=0x%02X:%lu sC=0x%02X:%lu sD=0x%02X:%lu sE=0x%02X:%lu sF=0x%02X:%lu)\n",
                          static_cast<unsigned long>(leaf120Ref00StateCorrectedMatches),
                          static_cast<unsigned long>(leaf120CrcSamples),
                          static_cast<unsigned long>(leaf120Ref00Fixed8Matches),
                          static_cast<unsigned long>(leaf120CrcSamples),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[0]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[0]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[1]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[1]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[2]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[2]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[3]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[3]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[4]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[4]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[5]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[5]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[6]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[6]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[7]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[7]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[8]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[8]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[9]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[9]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[10]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[10]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[11]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[11]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[12]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[12]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[13]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[13]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[14]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[14]),
                          static_cast<unsigned>(leaf120Ref00TopResidueByState[15]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[15]));
            Serial0.printf("[VCM-120-CRC2-STATS] ref00_state_fix(match=%lu/%lu fixed8=%lu/%lu s0=0x%02X:%lu s1=0x%02X:%lu s2=0x%02X:%lu s3=0x%02X:%lu s4=0x%02X:%lu s5=0x%02X:%lu s6=0x%02X:%lu s7=0x%02X:%lu s8=0x%02X:%lu s9=0x%02X:%lu sA=0x%02X:%lu sB=0x%02X:%lu sC=0x%02X:%lu sD=0x%02X:%lu sE=0x%02X:%lu sF=0x%02X:%lu)\n",
                           static_cast<unsigned long>(leaf120Ref00StateCorrectedMatches),
                           static_cast<unsigned long>(leaf120CrcSamples),
                           static_cast<unsigned long>(leaf120Ref00Fixed8Matches),
                           static_cast<unsigned long>(leaf120CrcSamples),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[0]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[0]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[1]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[1]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[2]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[2]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[3]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[3]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[4]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[4]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[5]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[5]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[6]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[6]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[7]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[7]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[8]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[8]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[9]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[9]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[10]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[10]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[11]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[11]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[12]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[12]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[13]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[13]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[14]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[14]),
                           static_cast<unsigned>(leaf120Ref00TopResidueByState[15]), static_cast<unsigned long>(leaf120Ref00TopResidueCountByState[15]));
        #endif
        #if METASENSE_LEAF_CRC_DEEP_LOGS
            uint32_t nibbleTopCount = 0U;
            uint8_t nibbleTop = 0U;
            for (uint8_t nib = 0U; nib < 16U; ++nib) {
                if (leaf120CrcNibbleSamples[nib] > nibbleTopCount) {
                    nibbleTopCount = leaf120CrcNibbleSamples[nib];
                    nibbleTop = nib;
                }
            }
            const float crcBrake0StateFixPct = (leaf120CrcBrakeSamples[0] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatchesByBrake[0]) / static_cast<float>(leaf120CrcBrakeSamples[0]))
                : 0.0f;
            const float crcBrake1StateFixPct = (leaf120CrcBrakeSamples[1] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatchesByBrake[1]) / static_cast<float>(leaf120CrcBrakeSamples[1]))
                : 0.0f;
            const float crcBrake0Fixed8Pct = (leaf120CrcBrakeSamples[0] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8MatchesByBrake[0]) / static_cast<float>(leaf120CrcBrakeSamples[0]))
                : 0.0f;
            const float crcBrake1Fixed8Pct = (leaf120CrcBrakeSamples[1] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8MatchesByBrake[1]) / static_cast<float>(leaf120CrcBrakeSamples[1]))
                : 0.0f;
            const float crcBrake0IdLoPct = (leaf120CrcBrakeSamples[0] > 0U)
                ? (100.0f * static_cast<float>(leaf120CrcRefIdLoMatchesByBrake[0]) / static_cast<float>(leaf120CrcBrakeSamples[0]))
                : 0.0f;
            const float crcBrake1IdLoPct = (leaf120CrcBrakeSamples[1] > 0U)
                ? (100.0f * static_cast<float>(leaf120CrcRefIdLoMatchesByBrake[1]) / static_cast<float>(leaf120CrcBrakeSamples[1]))
                : 0.0f;
            const float nibbleTopStateFixPct = (leaf120CrcNibbleSamples[nibbleTop] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatchesByNibble[nibbleTop]) / static_cast<float>(leaf120CrcNibbleSamples[nibbleTop]))
                : 0.0f;
            const float nibbleTopFixed8Pct = (leaf120CrcNibbleSamples[nibbleTop] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8MatchesByNibble[nibbleTop]) / static_cast<float>(leaf120CrcNibbleSamples[nibbleTop]))
                : 0.0f;
            const float nibbleTopIdLoPct = (leaf120CrcNibbleSamples[nibbleTop] > 0U)
                ? (100.0f * static_cast<float>(leaf120CrcRefIdLoMatchesByNibble[nibbleTop]) / static_cast<float>(leaf120CrcNibbleSamples[nibbleTop]))
                : 0.0f;
            uint8_t motorHiTop = 0U;
            uint32_t motorHiTopCount = 0U;
            for (uint8_t hi = 0U; hi < 16U; ++hi) {
                if (leaf120CrcMotorHiNibSamples[hi] > motorHiTopCount) {
                    motorHiTopCount = leaf120CrcMotorHiNibSamples[hi];
                    motorHiTop = hi;
                }
            }
            const float motorHiTopStateFixPct = (motorHiTopCount > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatchesByMotorHiNib[motorHiTop]) / static_cast<float>(motorHiTopCount))
                : 0.0f;
            const float motorHiTopFixed8Pct = (motorHiTopCount > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8MatchesByMotorHiNib[motorHiTop]) / static_cast<float>(motorHiTopCount))
                : 0.0f;
            const float motorHiTopIdLoPct = (motorHiTopCount > 0U)
                ? (100.0f * static_cast<float>(leaf120CrcRefIdLoMatchesByMotorHiNib[motorHiTop]) / static_cast<float>(motorHiTopCount))
                : 0.0f;
            const float crcCmdZeroStateFixPct = (leaf120CrcCmdStateSamples[0] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatchesByCmdState[0]) / static_cast<float>(leaf120CrcCmdStateSamples[0]))
                : 0.0f;
            const float crcCmdMotorStateFixPct = (leaf120CrcCmdStateSamples[1] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatchesByCmdState[1]) / static_cast<float>(leaf120CrcCmdStateSamples[1]))
                : 0.0f;
            const float crcCmdRegenStateFixPct = (leaf120CrcCmdStateSamples[2] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00StateCorrectedMatchesByCmdState[2]) / static_cast<float>(leaf120CrcCmdStateSamples[2]))
                : 0.0f;
            const float crcCmdZeroFixed8Pct = (leaf120CrcCmdStateSamples[0] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8MatchesByCmdState[0]) / static_cast<float>(leaf120CrcCmdStateSamples[0]))
                : 0.0f;
            const float crcCmdMotorFixed8Pct = (leaf120CrcCmdStateSamples[1] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8MatchesByCmdState[1]) / static_cast<float>(leaf120CrcCmdStateSamples[1]))
                : 0.0f;
            const float crcCmdRegenFixed8Pct = (leaf120CrcCmdStateSamples[2] > 0U)
                ? (100.0f * static_cast<float>(leaf120Ref00Fixed8MatchesByCmdState[2]) / static_cast<float>(leaf120CrcCmdStateSamples[2]))
                : 0.0f;
            const float crcCmdZeroIdLoPct = (leaf120CrcCmdStateSamples[0] > 0U)
                ? (100.0f * static_cast<float>(leaf120CrcRefIdLoMatchesByCmdState[0]) / static_cast<float>(leaf120CrcCmdStateSamples[0]))
                : 0.0f;
            const float crcCmdMotorIdLoPct = (leaf120CrcCmdStateSamples[1] > 0U)
                ? (100.0f * static_cast<float>(leaf120CrcRefIdLoMatchesByCmdState[1]) / static_cast<float>(leaf120CrcCmdStateSamples[1]))
                : 0.0f;
            const float crcCmdRegenIdLoPct = (leaf120CrcCmdStateSamples[2] > 0U)
                ? (100.0f * static_cast<float>(leaf120CrcRefIdLoMatchesByCmdState[2]) / static_cast<float>(leaf120CrcCmdStateSamples[2]))
                : 0.0f;
            Serial.printf("[VCM-120-CRC-SPLIT] brake(b0=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%,b1=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%) cmd(z=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%,m=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%,r=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%) motorHi(top=h%X n=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%) nib(top=n%X n=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%)\n",
                          static_cast<unsigned long>(leaf120CrcBrakeSamples[0]),
                          crcBrake0StateFixPct,
                          crcBrake0Fixed8Pct,
                          crcBrake0IdLoPct,
                          static_cast<unsigned long>(leaf120CrcBrakeSamples[1]),
                          crcBrake1StateFixPct,
                          crcBrake1Fixed8Pct,
                          crcBrake1IdLoPct,
                          static_cast<unsigned long>(leaf120CrcCmdStateSamples[0]),
                          crcCmdZeroStateFixPct,
                          crcCmdZeroFixed8Pct,
                          crcCmdZeroIdLoPct,
                          static_cast<unsigned long>(leaf120CrcCmdStateSamples[1]),
                          crcCmdMotorStateFixPct,
                          crcCmdMotorFixed8Pct,
                          crcCmdMotorIdLoPct,
                          static_cast<unsigned long>(leaf120CrcCmdStateSamples[2]),
                          crcCmdRegenStateFixPct,
                          crcCmdRegenFixed8Pct,
                          crcCmdRegenIdLoPct,
                          static_cast<unsigned>(motorHiTop),
                          static_cast<unsigned long>(motorHiTopCount),
                          motorHiTopStateFixPct,
                          motorHiTopFixed8Pct,
                          motorHiTopIdLoPct,
                          static_cast<unsigned>(nibbleTop),
                          static_cast<unsigned long>(nibbleTopCount),
                          nibbleTopStateFixPct,
                          nibbleTopFixed8Pct,
                          nibbleTopIdLoPct);
            Serial0.printf("[VCM-120-CRC-SPLIT] brake(b0=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%,b1=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%) cmd(z=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%,m=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%,r=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%) motorHi(top=h%X n=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%) nib(top=n%X n=%lu sf=%.1f%% f8=%.1f%% idlo=%.1f%%)\n",
                           static_cast<unsigned long>(leaf120CrcBrakeSamples[0]),
                           crcBrake0StateFixPct,
                           crcBrake0Fixed8Pct,
                           crcBrake0IdLoPct,
                           static_cast<unsigned long>(leaf120CrcBrakeSamples[1]),
                           crcBrake1StateFixPct,
                           crcBrake1Fixed8Pct,
                           crcBrake1IdLoPct,
                           static_cast<unsigned long>(leaf120CrcCmdStateSamples[0]),
                           crcCmdZeroStateFixPct,
                           crcCmdZeroFixed8Pct,
                           crcCmdZeroIdLoPct,
                           static_cast<unsigned long>(leaf120CrcCmdStateSamples[1]),
                           crcCmdMotorStateFixPct,
                           crcCmdMotorFixed8Pct,
                           crcCmdMotorIdLoPct,
                           static_cast<unsigned long>(leaf120CrcCmdStateSamples[2]),
                           crcCmdRegenStateFixPct,
                           crcCmdRegenFixed8Pct,
                           crcCmdRegenIdLoPct,
                           static_cast<unsigned>(motorHiTop),
                           static_cast<unsigned long>(motorHiTopCount),
                           motorHiTopStateFixPct,
                           motorHiTopFixed8Pct,
                           motorHiTopIdLoPct,
                           static_cast<unsigned>(nibbleTop),
                           static_cast<unsigned long>(nibbleTopCount),
                           nibbleTopStateFixPct,
                           nibbleTopFixed8Pct,
                           nibbleTopIdLoPct);

            auto bitSetPct = [](const uint32_t* counts, uint32_t samples, uint8_t bit) -> float {
                if (samples == 0U) {
                    return 0.0f;
                }
                uint32_t setCount = 0U;
                for (uint16_t value = 0U; value < 256U; ++value) {
                    if ((static_cast<uint8_t>(value) & static_cast<uint8_t>(1U << bit)) != 0U) {
                        setCount += counts[value];
                    }
                }
                return 100.0f * static_cast<float>(setCount) / static_cast<float>(samples);
            };

            const float posB2Pct = bitSetPct(leaf120B2PosCounts, leaf120B2PosSamples, 2U);
            const float posB3Pct = bitSetPct(leaf120B2PosCounts, leaf120B2PosSamples, 3U);
            const float posB4Pct = bitSetPct(leaf120B2PosCounts, leaf120B2PosSamples, 4U);
            const float posB5Pct = bitSetPct(leaf120B2PosCounts, leaf120B2PosSamples, 5U);
            const float negB2Pct = bitSetPct(leaf120B2NegCounts, leaf120B2NegSamples, 2U);
            const float negB3Pct = bitSetPct(leaf120B2NegCounts, leaf120B2NegSamples, 3U);
            const float negB4Pct = bitSetPct(leaf120B2NegCounts, leaf120B2NegSamples, 4U);
            const float negB5Pct = bitSetPct(leaf120B2NegCounts, leaf120B2NegSamples, 5U);

            float maxSepPct = 0.0f;
            uint8_t maxSepBit = 2U;
            const float sepB2 = fabsf(posB2Pct - negB2Pct);
            if (sepB2 > maxSepPct) {
                maxSepPct = sepB2;
                maxSepBit = 2U;
            }
            const float sepB3 = fabsf(posB3Pct - negB3Pct);
            if (sepB3 > maxSepPct) {
                maxSepPct = sepB3;
                maxSepBit = 3U;
            }
            const float sepB4 = fabsf(posB4Pct - negB4Pct);
            if (sepB4 > maxSepPct) {
                maxSepPct = sepB4;
                maxSepBit = 4U;
            }
            const float sepB5 = fabsf(posB5Pct - negB5Pct);
            if (sepB5 > maxSepPct) {
                maxSepPct = sepB5;
                maxSepBit = 5U;
            }

            const bool signBucketsReady = (leaf120B2PosSamples >= 200U) && (leaf120B2NegSamples >= 200U);
            const bool strongDirectionFlagBit = signBucketsReady && (maxSepPct >= 95.0f);
            const char* signOnlyVerdict = strongDirectionFlagBit ? "NO" : "YES";
            const char* idleZeroVerdict = (leaf120B2ZeroSamples >= 500U) ? "YES" : "UNCLEAR";

            Serial.printf("[VCM-120-SIGN-HYP] sign_only=%s idle_zero=%s sep(max=b%u %.1f%%)"
                          " pos(n=%lu b2=%.1f%% b3=%.1f%% b4=%.1f%% b5=%.1f%%)"
                          " neg(n=%lu b2=%.1f%% b3=%.1f%% b4=%.1f%% b5=%.1f%%)"
                          " zero(n=%lu)\n",
                          signOnlyVerdict,
                          idleZeroVerdict,
                          static_cast<unsigned>(maxSepBit),
                          maxSepPct,
                          static_cast<unsigned long>(leaf120B2PosSamples),
                          posB2Pct,
                          posB3Pct,
                          posB4Pct,
                          posB5Pct,
                          static_cast<unsigned long>(leaf120B2NegSamples),
                          negB2Pct,
                          negB3Pct,
                          negB4Pct,
                          negB5Pct,
                          static_cast<unsigned long>(leaf120B2ZeroSamples));
            Serial0.printf("[VCM-120-SIGN-HYP] sign_only=%s idle_zero=%s sep(max=b%u %.1f%%)"
                           " pos(n=%lu b2=%.1f%% b3=%.1f%% b4=%.1f%% b5=%.1f%%)"
                           " neg(n=%lu b2=%.1f%% b3=%.1f%% b4=%.1f%% b5=%.1f%%)"
                           " zero(n=%lu)\n",
                           signOnlyVerdict,
                           idleZeroVerdict,
                           static_cast<unsigned>(maxSepBit),
                           maxSepPct,
                           static_cast<unsigned long>(leaf120B2PosSamples),
                           posB2Pct,
                           posB3Pct,
                           posB4Pct,
                           posB5Pct,
                           static_cast<unsigned long>(leaf120B2NegSamples),
                           negB2Pct,
                           negB3Pct,
                           negB4Pct,
                           negB5Pct,
                           static_cast<unsigned long>(leaf120B2ZeroSamples));

            static const char* kScopeNames[kLeaf120MotorScopeCount] = {
                "d012_lsbB8",
                "idlo_d012_lsbB8",
                "idbe_d012_lsbB8",
                "idlo_d012_msb1D"
            };
            uint8_t bestScopeIdx = 0U;
            float bestScopeSfPct = -1.0f;
            float scopeDirectPct[kLeaf120MotorScopeCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            float scopeStateFixPct[kLeaf120MotorScopeCount] = {0.0f, 0.0f, 0.0f, 0.0f};
            for (uint8_t scopeIdx = 0U; scopeIdx < kLeaf120MotorScopeCount; ++scopeIdx) {
                const uint32_t n = leaf120MotorScopeSamples[scopeIdx];
                if (n > 0U) {
                    scopeDirectPct[scopeIdx] = 100.0f * static_cast<float>(leaf120MotorScopeDirectMatches[scopeIdx]) /
                                               static_cast<float>(n);
                    scopeStateFixPct[scopeIdx] = 100.0f * static_cast<float>(leaf120MotorScopeStateFixMatches[scopeIdx]) /
                                                 static_cast<float>(n);
                }
                if (scopeStateFixPct[scopeIdx] > bestScopeSfPct) {
                    bestScopeSfPct = scopeStateFixPct[scopeIdx];
                    bestScopeIdx = scopeIdx;
                }
            }

            Serial.printf("[VCM-120-MOTOR-SWEEP] best=%s sf=%.1f%% n=%lu"
                          " s0(d=%.1f%% sf=%.1f%%) s1(d=%.1f%% sf=%.1f%%)"
                          " s2(d=%.1f%% sf=%.1f%%) s3(d=%.1f%% sf=%.1f%%)\n",
                          kScopeNames[bestScopeIdx],
                          bestScopeSfPct,
                          static_cast<unsigned long>(leaf120MotorScopeSamples[bestScopeIdx]),
                          scopeDirectPct[0],
                          scopeStateFixPct[0],
                          scopeDirectPct[1],
                          scopeStateFixPct[1],
                          scopeDirectPct[2],
                          scopeStateFixPct[2],
                          scopeDirectPct[3],
                          scopeStateFixPct[3]);
            Serial0.printf("[VCM-120-MOTOR-SWEEP] best=%s sf=%.1f%% n=%lu"
                           " s0(d=%.1f%% sf=%.1f%%) s1(d=%.1f%% sf=%.1f%%)"
                           " s2(d=%.1f%% sf=%.1f%%) s3(d=%.1f%% sf=%.1f%%)\n",
                           kScopeNames[bestScopeIdx],
                           bestScopeSfPct,
                           static_cast<unsigned long>(leaf120MotorScopeSamples[bestScopeIdx]),
                           scopeDirectPct[0],
                           scopeStateFixPct[0],
                           scopeDirectPct[1],
                           scopeStateFixPct[1],
                           scopeDirectPct[2],
                           scopeStateFixPct[2],
                           scopeDirectPct[3],
                           scopeStateFixPct[3]);
            Serial.printf("[VCM-1DA-CRC-STATS] frames=%lu match(xor=%lu,sum=%lu,inv=%lu,c07_00=%lu,c07_FF=%lu,c1D_FF=%lu,c2F_FF=%lu,c1D_00=%lu,c1D_FF_00=%lu,c1D_id1dabe=%lu,c1D_id1dalo=%lu,c1D_ref_FF=%lu,c1D_ref_FF_00=%lu,c1D_ref_00=%lu,c1D_ref_id1dabe=%lu,c1D_ref_id1dalo=%lu,c2F_ref_FF=%lu)\n",
                          static_cast<unsigned long>(leaf1daCrcSamples),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[0]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[1]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[2]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[3]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[4]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[5]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[6]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[7]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[8]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[9]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[10]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[11]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[12]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[13]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[14]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[15]),
                          static_cast<unsigned long>(leaf1daCrcCandidateMatches[16]));
            Serial0.printf("[VCM-1DA-CRC-STATS] frames=%lu match(xor=%lu,sum=%lu,inv=%lu,c07_00=%lu,c07_FF=%lu,c1D_FF=%lu,c2F_FF=%lu,c1D_00=%lu,c1D_FF_00=%lu,c1D_id1dabe=%lu,c1D_id1dalo=%lu,c1D_ref_FF=%lu,c1D_ref_FF_00=%lu,c1D_ref_00=%lu,c1D_ref_id1dabe=%lu,c1D_ref_id1dalo=%lu,c2F_ref_FF=%lu)\n",
                           static_cast<unsigned long>(leaf1daCrcSamples),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[0]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[1]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[2]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[3]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[4]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[5]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[6]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[7]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[8]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[9]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[10]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[11]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[12]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[13]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[14]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[15]),
                           static_cast<unsigned long>(leaf1daCrcCandidateMatches[16]));
            uint32_t resTopCountMsb07 = 0U;
            uint32_t resTopCountMsb1D = 0U;
            uint32_t resTopCountMsb2F = 0U;
            uint32_t resTopCountLsb1D = 0U;
            uint32_t resTopCountLsb2F = 0U;
            const uint8_t resTopMsb07 = findTopByteValue(leaf1daResidueCounts[0], resTopCountMsb07);
            const uint8_t resTopMsb1D = findTopByteValue(leaf1daResidueCounts[2], resTopCountMsb1D);
            const uint8_t resTopMsb2F = findTopByteValue(leaf1daResidueCounts[4], resTopCountMsb2F);
            const uint8_t resTopLsb1D = findTopByteValue(leaf1daResidueCounts[5], resTopCountLsb1D);
            const uint8_t resTopLsb2F = findTopByteValue(leaf1daResidueCounts[7], resTopCountLsb2F);
            Serial.printf("[VCM-1DA-CRC2-STATS] frames=%lu residue(msb07=0x%02X:%lu,msb1D=0x%02X:%lu,msb2F=0x%02X:%lu,lsb1D=0x%02X:%lu,lsb2F=0x%02X:%lu) autosar(c2F_idbe=%lu,c1D_idbe=%lu,c2F_d6mask=%lu,c2F_clock=%lu)\n",
                          static_cast<unsigned long>(leaf1daResidueSamples),
                          static_cast<unsigned>(resTopMsb07),
                          static_cast<unsigned long>(resTopCountMsb07),
                          static_cast<unsigned>(resTopMsb1D),
                          static_cast<unsigned long>(resTopCountMsb1D),
                          static_cast<unsigned>(resTopMsb2F),
                          static_cast<unsigned long>(resTopCountMsb2F),
                          static_cast<unsigned>(resTopLsb1D),
                          static_cast<unsigned long>(resTopCountLsb1D),
                          static_cast<unsigned>(resTopLsb2F),
                          static_cast<unsigned long>(resTopCountLsb2F),
                          static_cast<unsigned long>(leaf1daAutosarCandidateMatches[0]),
                          static_cast<unsigned long>(leaf1daAutosarCandidateMatches[1]),
                          static_cast<unsigned long>(leaf1daAutosarCandidateMatches[2]),
                          static_cast<unsigned long>(leaf1daAutosarCandidateMatches[3]));
            Serial0.printf("[VCM-1DA-CRC2-STATS] frames=%lu residue(msb07=0x%02X:%lu,msb1D=0x%02X:%lu,msb2F=0x%02X:%lu,lsb1D=0x%02X:%lu,lsb2F=0x%02X:%lu) autosar(c2F_idbe=%lu,c1D_idbe=%lu,c2F_d6mask=%lu,c2F_clock=%lu)\n",
                           static_cast<unsigned long>(leaf1daResidueSamples),
                           static_cast<unsigned>(resTopMsb07),
                           static_cast<unsigned long>(resTopCountMsb07),
                           static_cast<unsigned>(resTopMsb1D),
                           static_cast<unsigned long>(resTopCountMsb1D),
                           static_cast<unsigned>(resTopMsb2F),
                           static_cast<unsigned long>(resTopCountMsb2F),
                           static_cast<unsigned>(resTopLsb1D),
                           static_cast<unsigned long>(resTopCountLsb1D),
                           static_cast<unsigned>(resTopLsb2F),
                           static_cast<unsigned long>(resTopCountLsb2F),
                           static_cast<unsigned long>(leaf1daAutosarCandidateMatches[0]),
                           static_cast<unsigned long>(leaf1daAutosarCandidateMatches[1]),
                           static_cast<unsigned long>(leaf1daAutosarCandidateMatches[2]),
                           static_cast<unsigned long>(leaf1daAutosarCandidateMatches[3]));
            uint32_t clkTopCountMsb07[4] = {0U, 0U, 0U, 0U};
            uint8_t clkTopMsb07[4] = {0U, 0U, 0U, 0U};
            uint32_t clkTopCountMsb1D[4] = {0U, 0U, 0U, 0U};
            uint8_t clkTopMsb1D[4] = {0U, 0U, 0U, 0U};
            for (uint8_t clk = 0U; clk < 4U; ++clk) {
                clkTopMsb07[clk] = findTopByteValue(leaf1daResidueByClock[0][clk], clkTopCountMsb07[clk]);
                clkTopMsb1D[clk] = findTopByteValue(leaf1daResidueByClock[2][clk], clkTopCountMsb1D[clk]);
            }
            Serial.printf("[VCM-1DA-CRC3-STATS] clock(n0=%lu,n1=%lu,n2=%lu,n3=%lu) msb07(c0=0x%02X:%lu,c1=0x%02X:%lu,c2=0x%02X:%lu,c3=0x%02X:%lu) msb1D(c0=0x%02X:%lu,c1=0x%02X:%lu,c2=0x%02X:%lu,c3=0x%02X:%lu)\n",
                          static_cast<unsigned long>(leaf1daClockSamples[0]),
                          static_cast<unsigned long>(leaf1daClockSamples[1]),
                          static_cast<unsigned long>(leaf1daClockSamples[2]),
                          static_cast<unsigned long>(leaf1daClockSamples[3]),
                          static_cast<unsigned>(clkTopMsb07[0]),
                          static_cast<unsigned long>(clkTopCountMsb07[0]),
                          static_cast<unsigned>(clkTopMsb07[1]),
                          static_cast<unsigned long>(clkTopCountMsb07[1]),
                          static_cast<unsigned>(clkTopMsb07[2]),
                          static_cast<unsigned long>(clkTopCountMsb07[2]),
                          static_cast<unsigned>(clkTopMsb07[3]),
                          static_cast<unsigned long>(clkTopCountMsb07[3]),
                          static_cast<unsigned>(clkTopMsb1D[0]),
                          static_cast<unsigned long>(clkTopCountMsb1D[0]),
                          static_cast<unsigned>(clkTopMsb1D[1]),
                          static_cast<unsigned long>(clkTopCountMsb1D[1]),
                          static_cast<unsigned>(clkTopMsb1D[2]),
                          static_cast<unsigned long>(clkTopCountMsb1D[2]),
                          static_cast<unsigned>(clkTopMsb1D[3]),
                          static_cast<unsigned long>(clkTopCountMsb1D[3]));
            Serial0.printf("[VCM-1DA-CRC3-STATS] clock(n0=%lu,n1=%lu,n2=%lu,n3=%lu) msb07(c0=0x%02X:%lu,c1=0x%02X:%lu,c2=0x%02X:%lu,c3=0x%02X:%lu) msb1D(c0=0x%02X:%lu,c1=0x%02X:%lu,c2=0x%02X:%lu,c3=0x%02X:%lu)\n",
                           static_cast<unsigned long>(leaf1daClockSamples[0]),
                           static_cast<unsigned long>(leaf1daClockSamples[1]),
                           static_cast<unsigned long>(leaf1daClockSamples[2]),
                           static_cast<unsigned long>(leaf1daClockSamples[3]),
                           static_cast<unsigned>(clkTopMsb07[0]),
                           static_cast<unsigned long>(clkTopCountMsb07[0]),
                           static_cast<unsigned>(clkTopMsb07[1]),
                           static_cast<unsigned long>(clkTopCountMsb07[1]),
                           static_cast<unsigned>(clkTopMsb07[2]),
                           static_cast<unsigned long>(clkTopCountMsb07[2]),
                           static_cast<unsigned>(clkTopMsb07[3]),
                           static_cast<unsigned long>(clkTopCountMsb07[3]),
                           static_cast<unsigned>(clkTopMsb1D[0]),
                           static_cast<unsigned long>(clkTopCountMsb1D[0]),
                           static_cast<unsigned>(clkTopMsb1D[1]),
                           static_cast<unsigned long>(clkTopCountMsb1D[1]),
                           static_cast<unsigned>(clkTopMsb1D[2]),
                           static_cast<unsigned long>(clkTopCountMsb1D[2]),
                           static_cast<unsigned>(clkTopMsb1D[3]),
                           static_cast<unsigned long>(clkTopCountMsb1D[3]));
        #endif
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
        #if METASENSE_LEAF_CRC_DEEP_LOGS
            Serial.printf("[VCM-1DA-CRC4-STATS] idlo_clock_fix(match=%lu/%lu c0=0x%02X:%lu c1=0x%02X:%lu c2=0x%02X:%lu c3=0x%02X:%lu)\n",
                          static_cast<unsigned long>(leaf1daIdLoClockCorrectedMatches),
                          static_cast<unsigned long>(leaf1daCrcSamples),
                          static_cast<unsigned>(leaf1daIdLoTopResidueByClock[0]),
                          static_cast<unsigned long>(leaf1daIdLoTopResidueCountByClock[0]),
                          static_cast<unsigned>(leaf1daIdLoTopResidueByClock[1]),
                          static_cast<unsigned long>(leaf1daIdLoTopResidueCountByClock[1]),
                          static_cast<unsigned>(leaf1daIdLoTopResidueByClock[2]),
                          static_cast<unsigned long>(leaf1daIdLoTopResidueCountByClock[2]),
                          static_cast<unsigned>(leaf1daIdLoTopResidueByClock[3]),
                          static_cast<unsigned long>(leaf1daIdLoTopResidueCountByClock[3]));
            Serial0.printf("[VCM-1DA-CRC4-STATS] idlo_clock_fix(match=%lu/%lu c0=0x%02X:%lu c1=0x%02X:%lu c2=0x%02X:%lu c3=0x%02X:%lu)\n",
                           static_cast<unsigned long>(leaf1daIdLoClockCorrectedMatches),
                           static_cast<unsigned long>(leaf1daCrcSamples),
                           static_cast<unsigned>(leaf1daIdLoTopResidueByClock[0]),
                           static_cast<unsigned long>(leaf1daIdLoTopResidueCountByClock[0]),
                           static_cast<unsigned>(leaf1daIdLoTopResidueByClock[1]),
                           static_cast<unsigned long>(leaf1daIdLoTopResidueCountByClock[1]),
                           static_cast<unsigned>(leaf1daIdLoTopResidueByClock[2]),
                           static_cast<unsigned long>(leaf1daIdLoTopResidueCountByClock[2]),
                           static_cast<unsigned>(leaf1daIdLoTopResidueByClock[3]),
                           static_cast<unsigned long>(leaf1daIdLoTopResidueCountByClock[3]));
        #endif
            lastCanAltDiagMs = now;
            lastCanAlt120FramesLogged = canStatsDiag.rx120Frames;
        }
    #if METASENSE_LEAF_VCM_LEGACY_VERBOSE_LOGS
        Serial.printf("[VCM-PRE] state=%s can_partner=%d can_ready=%d vcu_ready=%d inv_ready=%d inv_fault=%d inv_warn=%d inv_limp=%d hv=%.1f rpm=%.0f(age=%lu,n=%lu) tqNm=%.1f(age=%lu,n=%lu) temps=%lu(age=%lu,1DA=%lu,55A=%lu,legacy1DC=%lu,inv=%.1f,st_eff=%.1f,cool=%.1f,mot55A=%.1f,com55A=%.1f,igbt55A=%.1f,drv55A=%.1f) id120=%lu(age=%lu,len=%u,data=%02X %02X %02X %02X) id120cmd=%.2f(adj=%.2f,bias=%.2f,corr=%.3f,n=%lu) legacy55X=%lu(age=%lu,true55X_n=%lu,true55X_age=%lu,len=%u,src=0x%03lX,05B=%lu,50B=%lu,data=%02X %02X %02X %02X %02X %02X %02X %02X) legacy11A=%lu(age=%lu,len=%u,data=%02X %02X %02X %02X %02X %02X %02X %02X) hunt(std=%lu,ext=%lu,unk=%lu,last=0x%03lX,age=%lu,len=%u)\n",
                      leafVcmStateName(s_leafVcmState),
                      s_leafCanPartnerSeen ? 1 : 0,
                      MetaSense::CANBus::isReady() ? 1 : 0,
                      tele.vcuReady ? 1 : 0,
                      tele.leaf_invReady ? 1 : 0,
                      tele.leaf_invFault ? 1 : 0,
                      tele.leaf_invWarning ? 1 : 0,
                      tele.leaf_invLimp ? 1 : 0,
                      tele.vcuHvVoltage,
                      leafFbDiag.rpm,
                      id1daAgeMs,
                      static_cast<unsigned long>(leafFbDiag.rpm_frames),
                      leafFbDiag.torque_nm,
                      id1dbAgeMs,
                      static_cast<unsigned long>(leafFbDiag.torque_frames),
                      static_cast<unsigned long>(leafFbDiag.temps_frames),
                      id1dcAgeMs,
                      static_cast<unsigned long>(leafFbDiag.temps_1da_frames),
                      static_cast<unsigned long>(leafFbDiag.temps_1dc_frames),
                      static_cast<unsigned long>(leafFbDiag.temps_55a_frames),
                      leafFbDiag.inverter_temp,
                      leafFbDiag.stator_temp,
                      leafFbDiag.coolant_temp,
                      leafFbDiag.id55a_motor_temp_c,
                      leafFbDiag.id55a_com_board_temp_c,
                      leafFbDiag.id55a_igbt_temp_c,
                      leafFbDiag.id55a_driver_board_temp_c,
                      legacy55xCompat120Count,
                      legacy55xCompat120AgeMs,
                      static_cast<unsigned long>(canStatsDiag.rx55bFrames),
                      raw55bAgeMs,
                      static_cast<unsigned>(canStatsDiag.last55bLen),
                      static_cast<unsigned long>(canStatsDiag.last55bSrcId),
                      static_cast<unsigned long>(canStatsDiag.rx05bFrames),
                      static_cast<unsigned long>(canStatsDiag.rx50bFrames),
                      static_cast<unsigned>(canStatsDiag.last55bData[0]),
                      static_cast<unsigned>(canStatsDiag.last55bData[1]),
                      static_cast<unsigned>(canStatsDiag.last55bData[2]),
                      static_cast<unsigned>(canStatsDiag.last55bData[3]),
                      static_cast<unsigned>(canStatsDiag.last55bData[4]),
                      static_cast<unsigned>(canStatsDiag.last55bData[5]),
                      static_cast<unsigned>(canStatsDiag.last55bData[6]),
                      static_cast<unsigned>(canStatsDiag.last55bData[7]),
                      static_cast<unsigned long>(canStatsDiag.rx11aFrames),
                      raw11aAgeMs,
                      static_cast<unsigned>(canStatsDiag.last11aLen),
                      static_cast<unsigned>(canStatsDiag.last11aData[0]),
                      static_cast<unsigned>(canStatsDiag.last11aData[1]),
                      static_cast<unsigned>(canStatsDiag.last11aData[2]),
                      static_cast<unsigned>(canStatsDiag.last11aData[3]),
                      static_cast<unsigned>(canStatsDiag.last11aData[4]),
                      static_cast<unsigned>(canStatsDiag.last11aData[5]),
                      static_cast<unsigned>(canStatsDiag.last11aData[6]),
                      static_cast<unsigned>(canStatsDiag.last11aData[7]),
                      static_cast<unsigned long>(canStatsDiag.rx120Frames),
                      raw120AgeMs,
                      static_cast<unsigned>(canStatsDiag.last120Len),
                      static_cast<unsigned>(canStatsDiag.last120Data[0]),
                      static_cast<unsigned>(canStatsDiag.last120Data[1]),
                      static_cast<unsigned>(canStatsDiag.last120Data[2]),
                      static_cast<unsigned>(canStatsDiag.last120Data[3]),
                      id120CmdEstNmNow,
                      id120CmdAdjNmNow,
                      s_cmdZeroBiasNm,
                      beCorrNow,
                      static_cast<unsigned long>(s_corr120Be01Torque.n),
                      static_cast<unsigned long>(canStatsDiag.rxStdFrames),
                      static_cast<unsigned long>(canStatsDiag.rxExtFrames),
                      static_cast<unsigned long>(canStatsDiag.rxUnknownFrames),
                      static_cast<unsigned long>(canStatsDiag.lastUnknownId),
                      unkAgeMs,
                      static_cast<unsigned>(canStatsDiag.lastUnknownLen));
        Serial0.printf("[VCM-PRE] state=%s can_partner=%d can_ready=%d vcu_ready=%d inv_ready=%d inv_fault=%d inv_warn=%d inv_limp=%d hv=%.1f rpm=%.0f(age=%lu,n=%lu) tqNm=%.1f(age=%lu,n=%lu) temps=%lu(age=%lu,1DA=%lu,55A=%lu,legacy1DC=%lu,inv=%.1f,st_eff=%.1f,cool=%.1f,mot55A=%.1f,com55A=%.1f,igbt55A=%.1f,drv55A=%.1f) id120=%lu(age=%lu,len=%u,data=%02X %02X %02X %02X) id120cmd=%.2f(adj=%.2f,bias=%.2f,corr=%.3f,n=%lu) legacy55X=%lu(age=%lu,true55X_n=%lu,true55X_age=%lu,len=%u,src=0x%03lX,05B=%lu,50B=%lu,data=%02X %02X %02X %02X %02X %02X %02X %02X) legacy11A=%lu(age=%lu,len=%u,data=%02X %02X %02X %02X %02X %02X %02X %02X) hunt(std=%lu,ext=%lu,unk=%lu,last=0x%03lX,age=%lu,len=%u)\n",
                       leafVcmStateName(s_leafVcmState),
                       s_leafCanPartnerSeen ? 1 : 0,
                       MetaSense::CANBus::isReady() ? 1 : 0,
                       tele.vcuReady ? 1 : 0,
                       tele.leaf_invReady ? 1 : 0,
                       tele.leaf_invFault ? 1 : 0,
                       tele.leaf_invWarning ? 1 : 0,
                       tele.leaf_invLimp ? 1 : 0,
                       tele.vcuHvVoltage,
                       leafFbDiag.rpm,
                       id1daAgeMs,
                       static_cast<unsigned long>(leafFbDiag.rpm_frames),
                       leafFbDiag.torque_nm,
                       id1dbAgeMs,
                       static_cast<unsigned long>(leafFbDiag.torque_frames),
                       static_cast<unsigned long>(leafFbDiag.temps_frames),
                       id1dcAgeMs,
                       static_cast<unsigned long>(leafFbDiag.temps_1da_frames),
                       static_cast<unsigned long>(leafFbDiag.temps_1dc_frames),
                       static_cast<unsigned long>(leafFbDiag.temps_55a_frames),
                       leafFbDiag.inverter_temp,
                       leafFbDiag.stator_temp,
                       leafFbDiag.coolant_temp,
                       leafFbDiag.id55a_motor_temp_c,
                       leafFbDiag.id55a_com_board_temp_c,
                       leafFbDiag.id55a_igbt_temp_c,
                       leafFbDiag.id55a_driver_board_temp_c,
                       legacy55xCompat120Count,
                       legacy55xCompat120AgeMs,
                       static_cast<unsigned long>(canStatsDiag.rx55bFrames),
                       raw55bAgeMs,
                       static_cast<unsigned>(canStatsDiag.last55bLen),
                       static_cast<unsigned long>(canStatsDiag.last55bSrcId),
                       static_cast<unsigned long>(canStatsDiag.rx05bFrames),
                       static_cast<unsigned long>(canStatsDiag.rx50bFrames),
                       static_cast<unsigned>(canStatsDiag.last55bData[0]),
                       static_cast<unsigned>(canStatsDiag.last55bData[1]),
                       static_cast<unsigned>(canStatsDiag.last55bData[2]),
                       static_cast<unsigned>(canStatsDiag.last55bData[3]),
                       static_cast<unsigned>(canStatsDiag.last55bData[4]),
                       static_cast<unsigned>(canStatsDiag.last55bData[5]),
                       static_cast<unsigned>(canStatsDiag.last55bData[6]),
                       static_cast<unsigned>(canStatsDiag.last55bData[7]),
                       static_cast<unsigned long>(canStatsDiag.rx11aFrames),
                       raw11aAgeMs,
                       static_cast<unsigned>(canStatsDiag.last11aLen),
                       static_cast<unsigned>(canStatsDiag.last11aData[0]),
                       static_cast<unsigned>(canStatsDiag.last11aData[1]),
                       static_cast<unsigned>(canStatsDiag.last11aData[2]),
                       static_cast<unsigned>(canStatsDiag.last11aData[3]),
                       static_cast<unsigned>(canStatsDiag.last11aData[4]),
                       static_cast<unsigned>(canStatsDiag.last11aData[5]),
                       static_cast<unsigned>(canStatsDiag.last11aData[6]),
                       static_cast<unsigned>(canStatsDiag.last11aData[7]),
                       static_cast<unsigned long>(canStatsDiag.rx120Frames),
                       raw120AgeMs,
                       static_cast<unsigned>(canStatsDiag.last120Len),
                       static_cast<unsigned>(canStatsDiag.last120Data[0]),
                       static_cast<unsigned>(canStatsDiag.last120Data[1]),
                       static_cast<unsigned>(canStatsDiag.last120Data[2]),
                       static_cast<unsigned>(canStatsDiag.last120Data[3]),
                       id120CmdEstNmNow,
                       id120CmdAdjNmNow,
                       s_cmdZeroBiasNm,
                       beCorrNow,
                       static_cast<unsigned long>(s_corr120Be01Torque.n),
                       static_cast<unsigned long>(canStatsDiag.rxStdFrames),
                       static_cast<unsigned long>(canStatsDiag.rxExtFrames),
                       static_cast<unsigned long>(canStatsDiag.rxUnknownFrames),
                       static_cast<unsigned long>(canStatsDiag.lastUnknownId),
                       unkAgeMs,
                       static_cast<unsigned>(canStatsDiag.lastUnknownLen));
        Serial.printf("[VCM-CAND] id120(n=%lu,chg=%lu,mask=0x%02X/0x%02X,bchg=%lu,%lu,%lu,%lu) compat11A(n=%lu,chg=%lu,mask=0x%02X/0x%02X)\n",
                      static_cast<unsigned long>(canStatsDiag.rx120Frames),
                      static_cast<unsigned long>(canStatsDiag.rx120Changes),
                      static_cast<unsigned>(canStatsDiag.last120ChangeMask),
                  static_cast<unsigned>(canStatsDiag.agg120ChangeMask),
                  static_cast<unsigned long>(canStatsDiag.byteChg120[0]),
                  static_cast<unsigned long>(canStatsDiag.byteChg120[1]),
                  static_cast<unsigned long>(canStatsDiag.byteChg120[2]),
                  static_cast<unsigned long>(canStatsDiag.byteChg120[3]),
                      static_cast<unsigned long>(canStatsDiag.rx11aFrames),
                      static_cast<unsigned long>(canStatsDiag.rx11aChanges),
                      static_cast<unsigned>(canStatsDiag.last11aChangeMask),
                      static_cast<unsigned>(canStatsDiag.agg11aChangeMask));
        Serial0.printf("[VCM-CAND] id120(n=%lu,chg=%lu,mask=0x%02X/0x%02X,bchg=%lu,%lu,%lu,%lu) compat11A(n=%lu,chg=%lu,mask=0x%02X/0x%02X)\n",
                       static_cast<unsigned long>(canStatsDiag.rx120Frames),
                       static_cast<unsigned long>(canStatsDiag.rx120Changes),
                       static_cast<unsigned>(canStatsDiag.last120ChangeMask),
                   static_cast<unsigned>(canStatsDiag.agg120ChangeMask),
                   static_cast<unsigned long>(canStatsDiag.byteChg120[0]),
                   static_cast<unsigned long>(canStatsDiag.byteChg120[1]),
                   static_cast<unsigned long>(canStatsDiag.byteChg120[2]),
                   static_cast<unsigned long>(canStatsDiag.byteChg120[3]),
                       static_cast<unsigned long>(canStatsDiag.rx11aFrames),
                       static_cast<unsigned long>(canStatsDiag.rx11aChanges),
                       static_cast<unsigned>(canStatsDiag.last11aChangeMask),
                       static_cast<unsigned>(canStatsDiag.agg11aChangeMask));
#endif
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
#if METASENSE_LEAF_120_AUX_LOGS
        const uint8_t id120Sum = static_cast<uint8_t>(canStatsDiag.last120Data[0] +
                                   canStatsDiag.last120Data[1] +
                                   canStatsDiag.last120Data[2]);
        const uint8_t id120InvSum = static_cast<uint8_t>(~id120Sum);
        const bool torquePos = id120CmdAdjNmNow > 1.0f;
        const bool torqueNeg = id120CmdAdjNmNow < -1.0f;
        const bool torqueZero = !torquePos && !torqueNeg;
        const char* fnrGuess = torquePos ? "F" : (torqueNeg ? "R" : "N");
        const bool brakeActive = fabsf(tele.brakeTorqueNm) > 1.0f;
        const uint8_t id120B2 = canStatsDiag.last120Data[2];
        const uint8_t id120B2Hi2 = static_cast<uint8_t>((id120B2 >> 6) & 0x03U);
        const uint8_t id120B2Mid2 = static_cast<uint8_t>((id120B2 >> 4) & 0x03U);
        const uint8_t id120B2Lo2 = static_cast<uint8_t>(id120B2 & 0x03U);
        const uint8_t id120Payload[3] = {
            canStatsDiag.last120Data[0],
            canStatsDiag.last120Data[1],
            canStatsDiag.last120Data[2]
        };
        const uint8_t id120Crc07Init00 = crc8Msb(id120Payload, 3U, 0x07U, 0x00U, 0x00U);
        const uint8_t id120Crc07InitFF = crc8Msb(id120Payload, 3U, 0x07U, 0xFFU, 0x00U);
        const uint8_t id120Crc1DInitFF = crc8Msb(id120Payload, 3U, 0x1DU, 0xFFU, 0xFFU);
        const uint8_t id120Crc2FInitFF = crc8Msb(id120Payload, 3U, 0x2FU, 0xFFU, 0xFFU);
        const uint8_t id120Crc1DInit00 = crc8Msb(id120Payload, 3U, 0x1DU, 0x00U, 0x00U);
        const uint8_t id120Crc1DInitFFXor00 = crc8Msb(id120Payload, 3U, 0x1DU, 0xFFU, 0x00U);
        const uint8_t id120PayloadWithIdBe[5] = {
            0x01U,
            0x20U,
            canStatsDiag.last120Data[0],
            canStatsDiag.last120Data[1],
            canStatsDiag.last120Data[2]
        };
        const uint8_t id120PayloadWithIdLo[4] = {
            0x20U,
            canStatsDiag.last120Data[0],
            canStatsDiag.last120Data[1],
            canStatsDiag.last120Data[2]
        };
        const uint8_t id120Crc1DId120Be = crc8Msb(id120PayloadWithIdBe, 5U, 0x1DU, 0xFFU, 0xFFU);
        const uint8_t id120Crc1DId120Lo = crc8Msb(id120PayloadWithIdLo, 4U, 0x1DU, 0xFFU, 0xFFU);
        const uint8_t id120Crc1DRefInitFF = crc8Lsb(id120Payload, 3U, 0xB8U, 0xFFU, 0xFFU);
        const uint8_t id120Crc1DRefInitFFXor00 = crc8Lsb(id120Payload, 3U, 0xB8U, 0xFFU, 0x00U);
        const uint8_t id120Crc1DRefInit00 = crc8Lsb(id120Payload, 3U, 0xB8U, 0x00U, 0x00U);
        const uint8_t id120Crc1DRefId120Be = crc8Lsb(id120PayloadWithIdBe, 5U, 0xB8U, 0xFFU, 0xFFU);
        const uint8_t id120Crc1DRefId120Lo = crc8Lsb(id120PayloadWithIdLo, 4U, 0xB8U, 0xFFU, 0xFFU);
        const uint8_t id120Crc2FRefInitFF = crc8Lsb(id120Payload, 3U, 0xF4U, 0xFFU, 0xFFU);
        const uint32_t crcMatchMask = computeLeaf120CrcCandidateMask(canStatsDiag.last120Data,
                                          canStatsDiag.last120Len);
        const uint8_t id1daCrcRx = leafFbDiag.id1da_raw[7];
        const uint8_t id1daCrcXor = static_cast<uint8_t>(leafFbDiag.id1da_raw[0] ^
                                                         leafFbDiag.id1da_raw[1] ^
                                                         leafFbDiag.id1da_raw[2] ^
                                                         leafFbDiag.id1da_raw[3] ^
                                                         leafFbDiag.id1da_raw[4] ^
                                                         leafFbDiag.id1da_raw[5] ^
                                                         leafFbDiag.id1da_raw[6]);
        const uint8_t id1daCrcSum = static_cast<uint8_t>(leafFbDiag.id1da_raw[0] +
                                                         leafFbDiag.id1da_raw[1] +
                                                         leafFbDiag.id1da_raw[2] +
                                                         leafFbDiag.id1da_raw[3] +
                                                         leafFbDiag.id1da_raw[4] +
                                                         leafFbDiag.id1da_raw[5] +
                                                         leafFbDiag.id1da_raw[6]);
        const uint8_t id1daCrcInvSum = static_cast<uint8_t>(~id1daCrcSum);
        const uint8_t id1daCrc07Init00 = crc8Msb(leafFbDiag.id1da_raw, 7U, 0x07U, 0x00U, 0x00U);
        const uint8_t id1daCrc07InitFF = crc8Msb(leafFbDiag.id1da_raw, 7U, 0x07U, 0xFFU, 0x00U);
        const uint8_t id1daCrc1DInitFF = crc8Msb(leafFbDiag.id1da_raw, 7U, 0x1DU, 0xFFU, 0xFFU);
        const uint8_t id1daCrc2FInitFF = crc8Msb(leafFbDiag.id1da_raw, 7U, 0x2FU, 0xFFU, 0xFFU);
        const uint8_t id1daCrc1DInit00 = crc8Msb(leafFbDiag.id1da_raw, 7U, 0x1DU, 0x00U, 0x00U);
        const uint8_t id1daCrc1DInitFFXor00 = crc8Msb(leafFbDiag.id1da_raw, 7U, 0x1DU, 0xFFU, 0x00U);
        const uint8_t id1daPayloadWithIdBe[9] = {
            0x01U,
            0xDAU,
            leafFbDiag.id1da_raw[0],
            leafFbDiag.id1da_raw[1],
            leafFbDiag.id1da_raw[2],
            leafFbDiag.id1da_raw[3],
            leafFbDiag.id1da_raw[4],
            leafFbDiag.id1da_raw[5],
            leafFbDiag.id1da_raw[6]
        };
        const uint8_t id1daPayloadWithIdLo[8] = {
            0xDAU,
            leafFbDiag.id1da_raw[0],
            leafFbDiag.id1da_raw[1],
            leafFbDiag.id1da_raw[2],
            leafFbDiag.id1da_raw[3],
            leafFbDiag.id1da_raw[4],
            leafFbDiag.id1da_raw[5],
            leafFbDiag.id1da_raw[6]
        };
        const uint8_t id1daCrc1DIdBe = crc8Msb(id1daPayloadWithIdBe, 9U, 0x1DU, 0xFFU, 0xFFU);
        const uint8_t id1daCrc1DIdLo = crc8Msb(id1daPayloadWithIdLo, 8U, 0x1DU, 0xFFU, 0xFFU);
        const uint8_t id1daCrc1DRefInitFF = crc8Lsb(leafFbDiag.id1da_raw, 7U, 0xB8U, 0xFFU, 0xFFU);
        const uint8_t id1daCrc1DRefInitFFXor00 = crc8Lsb(leafFbDiag.id1da_raw, 7U, 0xB8U, 0xFFU, 0x00U);
        const uint8_t id1daCrc1DRefInit00 = crc8Lsb(leafFbDiag.id1da_raw, 7U, 0xB8U, 0x00U, 0x00U);
        const uint8_t id1daCrc1DRefIdBe = crc8Lsb(id1daPayloadWithIdBe, 9U, 0xB8U, 0xFFU, 0xFFU);
        const uint8_t id1daCrc1DRefIdLo = crc8Lsb(id1daPayloadWithIdLo, 8U, 0xB8U, 0xFFU, 0xFFU);
        const uint8_t id1daCrc2FRefInitFF = crc8Lsb(leafFbDiag.id1da_raw, 7U, 0xF4U, 0xFFU, 0xFFU);
        const uint32_t crc1daMatchMask = computeLeaf1daCrcCandidateMask(leafFbDiag.id1da_raw, 8U);
        uint8_t id1daResidues[kLeaf1daResidueCandidateCount] = {0U};
        computeLeaf1daResidues(leafFbDiag.id1da_raw, 8U, id1daResidues);
        const uint8_t id1daMgClock = static_cast<uint8_t>(leafFbDiag.id1da_raw[6] & 0x03U);
        const uint8_t id1daD6NoClock = static_cast<uint8_t>(leafFbDiag.id1da_raw[6] & 0xFCU);
        const uint8_t id1daAutoPayloadIdBe[9] = {
            0x01U, 0xDAU,
            leafFbDiag.id1da_raw[0],
            leafFbDiag.id1da_raw[1],
            leafFbDiag.id1da_raw[2],
            leafFbDiag.id1da_raw[3],
            leafFbDiag.id1da_raw[4],
            leafFbDiag.id1da_raw[5],
            leafFbDiag.id1da_raw[6]
        };
        const uint8_t id1daAutoPayloadD6Masked[9] = {
            leafFbDiag.id1da_raw[0],
            leafFbDiag.id1da_raw[1],
            leafFbDiag.id1da_raw[2],
            leafFbDiag.id1da_raw[3],
            leafFbDiag.id1da_raw[4],
            leafFbDiag.id1da_raw[5],
            id1daD6NoClock,
            0x01U,
            0xDAU
        };
        const uint8_t id1daAutoPayloadClockInjected[10] = {
            leafFbDiag.id1da_raw[0],
            leafFbDiag.id1da_raw[1],
            leafFbDiag.id1da_raw[2],
            leafFbDiag.id1da_raw[3],
            leafFbDiag.id1da_raw[4],
            leafFbDiag.id1da_raw[5],
            id1daD6NoClock,
            id1daMgClock,
            0x01U,
            0xDAU
        };
        const uint8_t id1daAutoC2FIdBe = crc8Msb(id1daAutoPayloadIdBe, 9U, 0x2FU, 0xFFU, 0xFFU);
        const uint8_t id1daAutoC1DIdBe = crc8Msb(id1daAutoPayloadIdBe, 9U, 0x1DU, 0xFFU, 0xFFU);
        const uint8_t id1daAutoC2FD6Masked = crc8Msb(id1daAutoPayloadD6Masked, 9U, 0x2FU, 0xFFU, 0xFFU);
        const uint8_t id1daAutoC2FClockInjected = crc8Msb(id1daAutoPayloadClockInjected, 10U, 0x2FU, 0xFFU, 0xFFU);
        const uint32_t crc1daAutosarMask = computeLeaf1daAutosarCandidateMask(leafFbDiag.id1da_raw, 8U);
#endif
    #if METASENSE_LEAF_MONITOR_SIMPLE_LOGS
        static uint32_t s_simpleLogDecimator = 0U;
        const char* tqModeStr = (tqCmpState == kTqCmpStateMotor) ? "MOTOR" :
                                ((tqCmpState == kTqCmpStateRegen) ? "REGEN" : "ZERO");
        const bool emitSimpleLine = (METASENSE_LEAF_MONITOR_SIMPLE_DECIMATE <= 1U) ||
                                    ((++s_simpleLogDecimator % METASENSE_LEAF_MONITOR_SIMPLE_DECIMATE) == 0U);
        if (emitSimpleLine) {
            Serial.printf("[VCM-SIMPLE] mode=%s cmd120=%.2fNm fb1da=%.2fNm delta=%.2fNm raw=%d crc=0x%02X ageMs(tq=%lu,120=%lu)\n",
                          tqModeStr,
                          id120CmdAdjNmNow,
                          tqEffNmNow,
                          tqErrAdjNmNow,
                          static_cast<int>(id120Sbe01),
                          static_cast<unsigned>(id120Crc),
                          id1dbAgeMs,
                          raw120AgeMs);
            Serial0.printf("[VCM-SIMPLE] mode=%s cmd120=%.2fNm fb1da=%.2fNm delta=%.2fNm raw=%d crc=0x%02X ageMs(tq=%lu,120=%lu)\n",
                           tqModeStr,
                           id120CmdAdjNmNow,
                           tqEffNmNow,
                           tqErrAdjNmNow,
                           static_cast<int>(id120Sbe01),
                           static_cast<unsigned>(id120Crc),
                           id1dbAgeMs,
                           raw120AgeMs);
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
    #if METASENSE_LEAF_120_AUX_LOGS
        Serial.printf("[VCM-120-AUX] fnr_guess=%s sign(p=%d,n=%d,z=%d) unknown_120_2=0x%02X slices(hi2=%u,mid2=%u,lo2=%u) brakeNm=%.2f brake=%d brkBits(b5=%u,b4=%u,b3=%u,b2=%u)\n",
                  fnrGuess,
                  torquePos ? 1 : 0,
                  torqueNeg ? 1 : 0,
                  torqueZero ? 1 : 0,
                  static_cast<unsigned>(id120B2),
                  static_cast<unsigned>(id120B2Hi2),
                  static_cast<unsigned>(id120B2Mid2),
                  static_cast<unsigned>(id120B2Lo2),
                  tele.brakeTorqueNm,
                  brakeActive ? 1 : 0,
                  static_cast<unsigned>((id120B2 >> 5) & 0x01U),
                  static_cast<unsigned>((id120B2 >> 4) & 0x01U),
                  static_cast<unsigned>((id120B2 >> 3) & 0x01U),
                  static_cast<unsigned>((id120B2 >> 2) & 0x01U));
        Serial0.printf("[VCM-120-AUX] fnr_guess=%s sign(p=%d,n=%d,z=%d) unknown_120_2=0x%02X slices(hi2=%u,mid2=%u,lo2=%u) brakeNm=%.2f brake=%d brkBits(b5=%u,b4=%u,b3=%u,b2=%u)\n",
                   fnrGuess,
                   torquePos ? 1 : 0,
                   torqueNeg ? 1 : 0,
                   torqueZero ? 1 : 0,
                   static_cast<unsigned>(id120B2),
                   static_cast<unsigned>(id120B2Hi2),
                   static_cast<unsigned>(id120B2Mid2),
                   static_cast<unsigned>(id120B2Lo2),
                   tele.brakeTorqueNm,
                   brakeActive ? 1 : 0,
                   static_cast<unsigned>((id120B2 >> 5) & 0x01U),
                   static_cast<unsigned>((id120B2 >> 4) & 0x01U),
                   static_cast<unsigned>((id120B2 >> 3) & 0x01U),
                   static_cast<unsigned>((id120B2 >> 2) & 0x01U));
        Serial.printf("[VCM-120-CRC] rx=0x%02X xor=0x%02X sum=0x%02X inv=0x%02X c07_00=0x%02X c07_FF=0x%02X c1D_FF=0x%02X c2F_FF=0x%02X c1D_00=0x%02X c1D_FF_00=0x%02X c1D_id120be=0x%02X c1D_id120lo=0x%02X c1D_ref_FF=0x%02X c1D_ref_FF_00=0x%02X c1D_ref_00=0x%02X c1D_ref_id120be=0x%02X c1D_ref_id120lo=0x%02X c2F_ref_FF=0x%02X matchMask=0x%05lX\n",
                  static_cast<unsigned>(id120Crc),
                  static_cast<unsigned>(id120Xor),
                  static_cast<unsigned>(id120Sum),
                  static_cast<unsigned>(id120InvSum),
                  static_cast<unsigned>(id120Crc07Init00),
                  static_cast<unsigned>(id120Crc07InitFF),
                  static_cast<unsigned>(id120Crc1DInitFF),
              static_cast<unsigned>(id120Crc2FInitFF),
              static_cast<unsigned>(id120Crc1DInit00),
              static_cast<unsigned>(id120Crc1DInitFFXor00),
              static_cast<unsigned>(id120Crc1DId120Be),
              static_cast<unsigned>(id120Crc1DId120Lo),
                  static_cast<unsigned>(id120Crc1DRefInitFF),
                  static_cast<unsigned>(id120Crc1DRefInitFFXor00),
                  static_cast<unsigned>(id120Crc1DRefInit00),
                  static_cast<unsigned>(id120Crc1DRefId120Be),
                  static_cast<unsigned>(id120Crc1DRefId120Lo),
                  static_cast<unsigned>(id120Crc2FRefInitFF),
                  static_cast<unsigned long>(crcMatchMask));
        Serial0.printf("[VCM-120-CRC] rx=0x%02X xor=0x%02X sum=0x%02X inv=0x%02X c07_00=0x%02X c07_FF=0x%02X c1D_FF=0x%02X c2F_FF=0x%02X c1D_00=0x%02X c1D_FF_00=0x%02X c1D_id120be=0x%02X c1D_id120lo=0x%02X c1D_ref_FF=0x%02X c1D_ref_FF_00=0x%02X c1D_ref_00=0x%02X c1D_ref_id120be=0x%02X c1D_ref_id120lo=0x%02X c2F_ref_FF=0x%02X matchMask=0x%05lX\n",
                   static_cast<unsigned>(id120Crc),
                   static_cast<unsigned>(id120Xor),
                   static_cast<unsigned>(id120Sum),
                   static_cast<unsigned>(id120InvSum),
                   static_cast<unsigned>(id120Crc07Init00),
                   static_cast<unsigned>(id120Crc07InitFF),
                   static_cast<unsigned>(id120Crc1DInitFF),
               static_cast<unsigned>(id120Crc2FInitFF),
               static_cast<unsigned>(id120Crc1DInit00),
               static_cast<unsigned>(id120Crc1DInitFFXor00),
               static_cast<unsigned>(id120Crc1DId120Be),
               static_cast<unsigned>(id120Crc1DId120Lo),
                   static_cast<unsigned>(id120Crc1DRefInitFF),
                   static_cast<unsigned>(id120Crc1DRefInitFFXor00),
                   static_cast<unsigned>(id120Crc1DRefInit00),
                   static_cast<unsigned>(id120Crc1DRefId120Be),
                   static_cast<unsigned>(id120Crc1DRefId120Lo),
                   static_cast<unsigned>(id120Crc2FRefInitFF),
                   static_cast<unsigned long>(crcMatchMask));
        Serial.printf("[VCM-1DA-CRC] rx=0x%02X xor=0x%02X sum=0x%02X inv=0x%02X c07_00=0x%02X c07_FF=0x%02X c1D_FF=0x%02X c2F_FF=0x%02X c1D_00=0x%02X c1D_FF_00=0x%02X c1D_id1dabe=0x%02X c1D_id1dalo=0x%02X c1D_ref_FF=0x%02X c1D_ref_FF_00=0x%02X c1D_ref_00=0x%02X c1D_ref_id1dabe=0x%02X c1D_ref_id1dalo=0x%02X c2F_ref_FF=0x%02X matchMask=0x%05lX\n",
                      static_cast<unsigned>(id1daCrcRx),
                      static_cast<unsigned>(id1daCrcXor),
                      static_cast<unsigned>(id1daCrcSum),
                      static_cast<unsigned>(id1daCrcInvSum),
                      static_cast<unsigned>(id1daCrc07Init00),
                      static_cast<unsigned>(id1daCrc07InitFF),
                      static_cast<unsigned>(id1daCrc1DInitFF),
                      static_cast<unsigned>(id1daCrc2FInitFF),
                      static_cast<unsigned>(id1daCrc1DInit00),
                      static_cast<unsigned>(id1daCrc1DInitFFXor00),
                      static_cast<unsigned>(id1daCrc1DIdBe),
                      static_cast<unsigned>(id1daCrc1DIdLo),
                      static_cast<unsigned>(id1daCrc1DRefInitFF),
                      static_cast<unsigned>(id1daCrc1DRefInitFFXor00),
                      static_cast<unsigned>(id1daCrc1DRefInit00),
                      static_cast<unsigned>(id1daCrc1DRefIdBe),
                      static_cast<unsigned>(id1daCrc1DRefIdLo),
                      static_cast<unsigned>(id1daCrc2FRefInitFF),
                      static_cast<unsigned long>(crc1daMatchMask));
        Serial0.printf("[VCM-1DA-CRC] rx=0x%02X xor=0x%02X sum=0x%02X inv=0x%02X c07_00=0x%02X c07_FF=0x%02X c1D_FF=0x%02X c2F_FF=0x%02X c1D_00=0x%02X c1D_FF_00=0x%02X c1D_id1dabe=0x%02X c1D_id1dalo=0x%02X c1D_ref_FF=0x%02X c1D_ref_FF_00=0x%02X c1D_ref_00=0x%02X c1D_ref_id1dabe=0x%02X c1D_ref_id1dalo=0x%02X c2F_ref_FF=0x%02X matchMask=0x%05lX\n",
                       static_cast<unsigned>(id1daCrcRx),
                       static_cast<unsigned>(id1daCrcXor),
                       static_cast<unsigned>(id1daCrcSum),
                       static_cast<unsigned>(id1daCrcInvSum),
                       static_cast<unsigned>(id1daCrc07Init00),
                       static_cast<unsigned>(id1daCrc07InitFF),
                       static_cast<unsigned>(id1daCrc1DInitFF),
                       static_cast<unsigned>(id1daCrc2FInitFF),
                       static_cast<unsigned>(id1daCrc1DInit00),
                       static_cast<unsigned>(id1daCrc1DInitFFXor00),
                       static_cast<unsigned>(id1daCrc1DIdBe),
                       static_cast<unsigned>(id1daCrc1DIdLo),
                       static_cast<unsigned>(id1daCrc1DRefInitFF),
                       static_cast<unsigned>(id1daCrc1DRefInitFFXor00),
                       static_cast<unsigned>(id1daCrc1DRefInit00),
                       static_cast<unsigned>(id1daCrc1DRefIdBe),
                       static_cast<unsigned>(id1daCrc1DRefIdLo),
                       static_cast<unsigned>(id1daCrc2FRefInitFF),
                       static_cast<unsigned long>(crc1daMatchMask));
                Serial.printf("[VCM-1DA-CRC2] d6=0x%02X clock=%u residue(msb07_00=0x%02X,msb1D_FF=0x%02X,msb2F_FF=0x%02X,lsb1D_FF=0x%02X,lsb2F_FF=0x%02X) autosar(c2F_idbe=0x%02X,c1D_idbe=0x%02X,c2F_d6mask=0x%02X,c2F_clock=0x%02X,mask=0x%01lX)\n",
                          static_cast<unsigned>(leafFbDiag.id1da_raw[6]),
                          static_cast<unsigned>(id1daMgClock),
                          static_cast<unsigned>(id1daResidues[0]),
                          static_cast<unsigned>(id1daResidues[2]),
                          static_cast<unsigned>(id1daResidues[4]),
                          static_cast<unsigned>(id1daResidues[5]),
                          static_cast<unsigned>(id1daResidues[7]),
                          static_cast<unsigned>(id1daAutoC2FIdBe),
                          static_cast<unsigned>(id1daAutoC1DIdBe),
                          static_cast<unsigned>(id1daAutoC2FD6Masked),
                          static_cast<unsigned>(id1daAutoC2FClockInjected),
                          static_cast<unsigned long>(crc1daAutosarMask));
                    Serial0.printf("[VCM-1DA-CRC2] d6=0x%02X clock=%u residue(msb07_00=0x%02X,msb1D_FF=0x%02X,msb2F_FF=0x%02X,lsb1D_FF=0x%02X,lsb2F_FF=0x%02X) autosar(c2F_idbe=0x%02X,c1D_idbe=0x%02X,c2F_d6mask=0x%02X,c2F_clock=0x%02X,mask=0x%01lX)\n",
                               static_cast<unsigned>(leafFbDiag.id1da_raw[6]),
                               static_cast<unsigned>(id1daMgClock),
                           static_cast<unsigned>(id1daResidues[0]),
                           static_cast<unsigned>(id1daResidues[2]),
                           static_cast<unsigned>(id1daResidues[4]),
                           static_cast<unsigned>(id1daResidues[5]),
                           static_cast<unsigned>(id1daResidues[7]),
                           static_cast<unsigned>(id1daAutoC2FIdBe),
                           static_cast<unsigned>(id1daAutoC1DIdBe),
                           static_cast<unsigned>(id1daAutoC2FD6Masked),
                           static_cast<unsigned>(id1daAutoC2FClockInjected),
                           static_cast<unsigned long>(crc1daAutosarMask));
    #endif
    #if METASENSE_LEAF_VCM_LEGACY_VERBOSE_LOGS
        Serial.printf("[VCM-120-DECODE] cmd_dig=%d cmdNm=%.2f cmdAdjNm=%.2f model=%s corr=%.3f n=%lu compat(le23=%u,legacy11a_tail=0x%04X)\n",
                  static_cast<int>(id120Sbe01),
                  id120CmdEstNmNow,
                  id120CmdAdjNmNow,
                  useFitModelNow ? "fit" : "base",
                  beCorrNow,
                  static_cast<unsigned long>(s_corr120Be01Torque.n),
                  static_cast<unsigned>(id120Le23),
                  static_cast<unsigned>(id11aTail));
        Serial0.printf("[VCM-120-DECODE] cmd_dig=%d cmdNm=%.2f cmdAdjNm=%.2f model=%s corr=%.3f n=%lu compat(le23=%u,legacy11a_tail=0x%04X)\n",
                   static_cast<int>(id120Sbe01),
                   id120CmdEstNmNow,
                   id120CmdAdjNmNow,
                   useFitModelNow ? "fit" : "base",
                   beCorrNow,
                   static_cast<unsigned long>(s_corr120Be01Torque.n),
                   static_cast<unsigned>(id120Le23),
                   static_cast<unsigned>(id11aTail));
        Serial.printf("[VCM-120-CMD] src=be01_s estNm=%.2f slope=%.6f offset=%.3f model=%s fit(acc=%lu,skip0=%lu,hold=%d)\n",
                  id120CmdEstNmNow,
                  beSlopeNow,
                  beOffsetNow,
                  useFitModelNow ? "fit" : "base",
                  static_cast<unsigned long>(s_corrFitAcceptedN),
                  static_cast<unsigned long>(s_corrFitZeroHoldSkipN),
                  s_corrFitFrozenNow ? 1 : 0);
        Serial0.printf("[VCM-120-CMD] src=be01_s estNm=%.2f slope=%.6f offset=%.3f model=%s fit(acc=%lu,skip0=%lu,hold=%d)\n",
                   id120CmdEstNmNow,
                   beSlopeNow,
                   beOffsetNow,
                   useFitModelNow ? "fit" : "base",
                   static_cast<unsigned long>(s_corrFitAcceptedN),
                   static_cast<unsigned long>(s_corrFitZeroHoldSkipN),
                   s_corrFitFrozenNow ? 1 : 0);
        Serial.printf("[VCM-CORR] n=%lu corr(be01_s,tq)=%.3f corr(le01_s,tq)=%.3f\n",
                  static_cast<unsigned long>(s_corr120Be01Torque.n),
                  corrValue(s_corr120Be01Torque),
                  corrValue(s_corr120Le01Torque));
        Serial0.printf("[VCM-CORR] n=%lu corr(be01_s,tq)=%.3f corr(le01_s,tq)=%.3f\n",
                   static_cast<unsigned long>(s_corr120Be01Torque.n),
                   corrValue(s_corr120Be01Torque),
                   corrValue(s_corr120Le01Torque));
    #endif
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
#if METASENSE_LEAF_120_NOISE_LOGS
                const uint32_t tqTotal = s_vcmTqFreshCount + s_vcmTqStaleCount;
                const uint32_t cmd120Total = s_vcm120FreshCount + s_vcm120StaleCount;
                const float tqFreshPct = (tqTotal > 0U)
                    ? (100.0f * static_cast<float>(s_vcmTqFreshCount) / static_cast<float>(tqTotal))
                    : 0.0f;
                const float cmd120FreshPct = (cmd120Total > 0U)
                    ? (100.0f * static_cast<float>(s_vcm120FreshCount) / static_cast<float>(cmd120Total))
                    : 0.0f;
                const float cmdAdjMinOut = s_vcmAdjNoiseInit ? s_vcmCmdAdjMinNm : 0.0f;
                const float cmdAdjMaxOut = s_vcmAdjNoiseInit ? s_vcmCmdAdjMaxNm : 0.0f;
                const float cmdAdjEmaOut = s_vcmAdjNoiseInit ? s_vcmCmdAdjEmaNm : 0.0f;
                Serial.printf("[VCM-120-NOISE] cmdRaw(min=%.2f max=%.2f ema=%.2f)"
                              " cmdAdj(min=%.2f max=%.2f ema=%.2f pos=%lu neg=%lu zero=%lu flips=%lu out=%lu clip=%.2f db=%.2f)"
                              " eff(ema=%.2f) fresh(tq=%lu/%lu %.1f%%,120=%lu/%lu %.1f%%)\n",
                              s_vcmCmdRawMinNm,
                              s_vcmCmdRawMaxNm,
                              s_vcmCmdRawEmaNm,
                              cmdAdjMinOut,
                              cmdAdjMaxOut,
                              cmdAdjEmaOut,
                              static_cast<unsigned long>(s_vcmCmdPosCount),
                              static_cast<unsigned long>(s_vcmCmdNegCount),
                              static_cast<unsigned long>(s_vcmCmdZeroCount),
                              static_cast<unsigned long>(s_vcmCmdFlipCount),
                              static_cast<unsigned long>(s_vcmCmdOutlierCount),
                              noiseOutlierAbsNm,
                              noiseDbNm,
                              s_vcmEffEmaNm,
                              static_cast<unsigned long>(s_vcmTqFreshCount),
                              static_cast<unsigned long>(tqTotal),
                              tqFreshPct,
                              static_cast<unsigned long>(s_vcm120FreshCount),
                              static_cast<unsigned long>(cmd120Total),
                              cmd120FreshPct);
                Serial0.printf("[VCM-120-NOISE] cmdRaw(min=%.2f max=%.2f ema=%.2f)"
                               " cmdAdj(min=%.2f max=%.2f ema=%.2f pos=%lu neg=%lu zero=%lu flips=%lu out=%lu clip=%.2f db=%.2f)"
                               " eff(ema=%.2f) fresh(tq=%lu/%lu %.1f%%,120=%lu/%lu %.1f%%)\n",
                               s_vcmCmdRawMinNm,
                               s_vcmCmdRawMaxNm,
                               s_vcmCmdRawEmaNm,
                               cmdAdjMinOut,
                               cmdAdjMaxOut,
                               cmdAdjEmaOut,
                               static_cast<unsigned long>(s_vcmCmdPosCount),
                               static_cast<unsigned long>(s_vcmCmdNegCount),
                               static_cast<unsigned long>(s_vcmCmdZeroCount),
                               static_cast<unsigned long>(s_vcmCmdFlipCount),
                               static_cast<unsigned long>(s_vcmCmdOutlierCount),
                               noiseOutlierAbsNm,
                               noiseDbNm,
                               s_vcmEffEmaNm,
                               static_cast<unsigned long>(s_vcmTqFreshCount),
                               static_cast<unsigned long>(tqTotal),
                               tqFreshPct,
                               static_cast<unsigned long>(s_vcm120FreshCount),
                               static_cast<unsigned long>(cmd120Total),
                               cmd120FreshPct);
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
#if METASENSE_LEAF_VCM_RX_WARN_LOGS
                    Serial.println("[VCM-RX] Waiting for Leaf partner frame 1DA (RPM) not seen yet");
                    Serial0.println("[VCM-RX] Waiting for Leaf partner frame 1DA (RPM) not seen yet");
#endif
                    s_leafRxAwaitPartnerLastMs = now;
                }
            } else if (s_leafRxWarnLastMs == 0U || (now - s_leafRxWarnLastMs) >= CAN_RX_MISSING_LOG_PERIOD_MS) {
#if METASENSE_LEAF_VCM_RX_WARN_LOGS
                Serial.printf("[VCM-RX] Missing/old required frame: 1DA=%d age_ms(1DA=%lu)\n",
                              id1daFresh ? 0 : 1,
                              static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.rpm_update_ms)));
                Serial0.printf("[VCM-RX] Missing/old required frame: 1DA=%d age_ms(1DA=%lu)\n",
                               id1daFresh ? 0 : 1,
                               static_cast<unsigned long>(elapsedMsSafe(now, leafFbDiag.rpm_update_ms)));
#endif
                s_leafRxWarnLastMs = now;
            }
        } else {
            s_leafRxWarnLastMs = 0U;
        }

        s_leafRxDiagLastMs = now;
    }

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
                const bool sent = MetaSense::Input::sendLeafTorqueCommand120(0.0f,
                                                                             false,
                                                                             false,
                                                                             false,
                                                                             true);
                logLeaf120ShadowFrame(now, 0.0f, false, false, false, true, sent);
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
        const bool id1daFreshForTorque = (aliveFb.rpm_update_ms != 0U) &&
                         (elapsedMsSafe(now, aliveFb.rpm_update_ms) <= CAN_RX_TARGET_MAX_AGE_MS);
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

        const bool torqueCommandUnlocked = safetyOk &&
                                           (s_leafVcmState == LeafVcmBringupState::Ready) &&
                                           id1daFreshForTorque &&
                                           s_leafCanPartnerSeen &&
                                           !s_leafSimFeedbackActive;
        if (!torqueCommandUnlocked && fabsf(torqueToSend) > 0.0f) {
            torqueToSend = 0.0f;
        }

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
                        Serial.printf("[VCM-GAPTEST] Pausing 0x120 TX for %lu ms to provoke timeout bit\n",
                                      static_cast<unsigned long>(METASENSE_LEAF_TX_GAP_TEST_DURATION_MS));
                        Serial0.printf("[VCM-GAPTEST] Pausing 0x120 TX for %lu ms to provoke timeout bit\n",
                                       static_cast<unsigned long>(METASENSE_LEAF_TX_GAP_TEST_DURATION_MS));
                        s_leafTxGapTestLoggedStart = true;
                    }
                } else {
                    s_leafTxGapTestActive = false;
                    if (!s_leafTxGapTestLoggedEnd) {
                        Serial.println("[VCM-GAPTEST] 0x120 TX pause ended");
                        Serial0.println("[VCM-GAPTEST] 0x120 TX pause ended");
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
        const bool txSent = sendLeafTorqueCommand120(torqueToSend,
                                 readyBit,
                                 hvOkBit,
                                 brakeBit,
                                 gearDriveBit);
        logLeaf120ShadowFrame(now,
                              torqueToSend,
                              readyBit,
                              hvOkBit,
                              brakeBit,
                              gearDriveBit,
                              txSent);
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
