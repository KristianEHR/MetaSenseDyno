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
#include "globals.h"
#include "TelnetSerialBridge.h"

// Default: CAN Monitor JSON telemetry is DISABLED (reduce WebSocket payload)
// Enable via platformio.ini: -D METASENSE_CAN_MONITOR_JSON_ENABLED=1
#ifndef METASENSE_CAN_MONITOR_JSON_ENABLED
#define METASENSE_CAN_MONITOR_JSON_ENABLED 0
#endif

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
static uint8_t s_leaf11aMuxTemplateBySel[4][8] = {{0U}};
// UI-controlled values for 0x11A frame (initialized with defaults from CanConfig.h)
static uint8_t s_leaf11aUiGear = METASENSE_LEAF_11A_FORCE_GEAR;
static uint8_t s_leaf11aUiCarOnOff = METASENSE_LEAF_11A_FORCE_CARONOFF;
static uint32_t lastLeaf1d4MonitorSampleMs = 0;
static uint32_t lastCanBusOffSeen = 0;
static uint32_t lastCanStatusQueryFailuresSeen = 0;

static const uint32_t CAN_TEMP_TIMEOUT_MS = 1000;
static const uint32_t CAN_TX_PERIOD_MS = 10;
static const uint32_t LEAF_1D4_TORQUE_PAYLOAD_UPDATE_PERIOD_MS = 10;  // Update torque payload every 10ms, send frame every 10ms
static const uint32_t LEAF_1D4_MONITOR_SAMPLE_PERIOD_MS = 100;
static const uint32_t CAN_RX_CHECK_PERIOD_MS = 20;
static const uint32_t CAN_RX_TARGET_MAX_AGE_MS = 250;
static const uint32_t CAN_RX_MISSING_LOG_PERIOD_MS = 5000;
static const uint32_t CAN_1DA_CRC_BAD_STREAK_LIMIT = 10;

// ─────────────────────────────────────────────────────────────────────────────
// Voltage-based brake torque controller (overvoltage protection)
// ─────────────────────────────────────────────────────────────────────────────
// When inverter voltage > 450V, increase brake torque proportionally to:
// - Dissipate more power (brake_power = V × I)
// - Keep inverter voltage under control during high inertia braking
// - Limit voltage overshoot and prevent inverter faults
static bool s_brakeTorqueControlActive = false;  // Tracks if protection is engaged (hysteresis)
static float s_computedBrakeTorqueNm = 0.0f;    // Output of voltage-based controller

// Voltage thresholds for hysteresis
static constexpr float kInvVoltageActivateThresholdV = 450.0f;   // Activate at 450V
static constexpr float kInvVoltageDeactivateThresholdV = 440.0f; // Release at 440V (avoid hunting)
static constexpr float kInvVoltageSafeMaxV = 500.0f;             // Scale ramp based on this max

// Computes brake torque as a linear ramp: 4Nm @ 450V → 30Nm @ 500V+
float computeVoltageBrakeTorque(float invVoltageV, float idleTorqueNm, float maxBrakeTorqueNm)
{
    // Hysteresis: activate at 450V, deactivate at 440V
    if (invVoltageV >= kInvVoltageActivateThresholdV) {
        s_brakeTorqueControlActive = true;
    } else if (invVoltageV < kInvVoltageDeactivateThresholdV) {
        s_brakeTorqueControlActive = false;
    }
    
    // When active and above activate threshold, ramp torque
    if (s_brakeTorqueControlActive && invVoltageV >= kInvVoltageActivateThresholdV) {
        const float voltageOverage = invVoltageV - kInvVoltageActivateThresholdV;
        const float voltageRampMax = kInvVoltageSafeMaxV - kInvVoltageActivateThresholdV;
        const float rampFraction = constrain(voltageOverage / voltageRampMax, 0.0f, 1.0f);
        const float torqueDelta = maxBrakeTorqueNm - idleTorqueNm;
        return idleTorqueNm + (torqueDelta * rampFraction);
    }
    
    // Return idle torque otherwise
    return idleTorqueNm;
}

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
#ifndef METASENSE_TORQUE_STEP_SEQUENCER_ENABLED
#define METASENSE_TORQUE_STEP_SEQUENCER_ENABLED 0
#endif
#ifndef METASENSE_TORQUE_STEP_SEQUENCER_STEP_NM
#define METASENSE_TORQUE_STEP_SEQUENCER_STEP_NM 0.5f
#endif
#ifndef METASENSE_TORQUE_STEP_SEQUENCER_MAX_NM
#define METASENSE_TORQUE_STEP_SEQUENCER_MAX_NM 3.0f
#endif
#ifndef METASENSE_TORQUE_STEP_SEQUENCER_DWELL_MS
#define METASENSE_TORQUE_STEP_SEQUENCER_DWELL_MS 5000U
#endif
#ifndef METASENSE_LEAF_120_CMD_BASE_SLOPE
#define METASENSE_LEAF_120_CMD_BASE_SLOPE 0.0300f
#endif
#ifndef METASENSE_LEAF_120_CMD_BASE_OFFSET_NM
#define METASENSE_LEAF_120_CMD_BASE_OFFSET_NM 0.000f
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
#ifndef METASENSE_LEAF_120_TX_COMMIT_ENABLED
#define METASENSE_LEAF_120_TX_COMMIT_ENABLED 0
#endif

#ifndef METASENSE_LEAF_120_STARTUP_ZERO_TORQUE
// Hold 0x120 torque demand at zero during initial HV bring-up so the inverter
// can precharge cleanly and clear its error bits before any torque is applied.
// A future sequencer can replace this with a staged ramp without changing the
// surrounding state-machine logic.
#define METASENSE_LEAF_120_STARTUP_ZERO_TORQUE 1
#endif
#ifndef METASENSE_LEAF_CRC_CANDIDATE_HUNT
// Disable reverse-engineering candidate sweeps in normal operation.
#define METASENSE_LEAF_CRC_CANDIDATE_HUNT 0
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
static uint32_t s_leafRxDiagLastMs = 0;
static uint32_t s_leafRxWarnLastMs = 0;
static uint32_t s_leafRxAwaitPartnerLastMs = 0;
static uint32_t s_leafTxGapTestCycleStartMs = 0;
static bool s_leafTxGapTestActive = false;
static bool s_leafTxGapTestLoggedStart = false;
static bool s_leafTxGapTestLoggedEnd = false;
static volatile float s_leafUiTorqueDemandNm = 0.0f;
static volatile bool s_leafManualTorqueMode = true;  // true=manual (default), false=auto (PI controller)
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
#define METASENSE_LEAF_CAN_TX_ENABLED 1
#endif
#ifndef METASENSE_LEAF_CAN_LISTEN_ONLY
#define METASENSE_LEAF_CAN_LISTEN_ONLY 0
#endif
#ifndef METASENSE_LEAF_CAN_HANDSHAKE_ON_FIRST_1DA
#define METASENSE_LEAF_CAN_HANDSHAKE_ON_FIRST_1DA 0
#endif
// All 0x11A template definitions are now in include/CanConfig.h
// These will be used from CanConfig.h via #ifndef guards
#ifndef METASENSE_55A
#define METASENSE_55A 0
#endif
#ifndef METASENSE_LEAF_CAN_VARIANT_READY_FALLBACK
#define METASENSE_LEAF_CAN_VARIANT_READY_FALLBACK 1
#endif
#ifndef METASENSE_LEAF_1D4_CRC_CLOCK_XOR_ENABLE
// Conformant 0x1D4 CRC implementation:
// CRC8 MSB poly 0x1D over [idLo + payload7], then XOR by HCM clock bin.
#define METASENSE_LEAF_1D4_CRC_CLOCK_XOR_ENABLE 1
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_TX_MODE
// 0: Build 0x1D4 command fields from logic.
// 1: Start from a known-good Thunderstruck template frame and only patch
//    selected fields (torque + clock + CRC) for controlled acceptance tests.
// 2: Replay a captured clock-continuous 0x1D4 frame loop exactly as generated
//    by tools/analyze_1d4_sniff.py --emit-loop-header.
// 3: Use newest captured 0x1D4 ring-buffer frame as TX base, then patch
//    torque + clock + CRC; if ring is empty, fallback to mode 0 builder.
// 4: Strict raw replay from 0x1D4 ring-buffer frame (no patching, no builder fallback).
#define METASENSE_LEAF_1D4_TEMPLATE_TX_MODE 0
#endif
#ifndef METASENSE_LEAF_1D4_RING_TX_SOURCE_AGE
// 0 reads newest ring frame, 1 previous, etc.
#define METASENSE_LEAF_1D4_RING_TX_SOURCE_AGE 0
#endif
#ifndef METASENSE_LEAF_1D4_REPLAY_RECALC_CRC
// Keep enabled to harden generated loop frames against accidental edits.
#define METASENSE_LEAF_1D4_REPLAY_RECALC_CRC 1
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_MAX_ABS_TORQUE_NM
// Keep template TX in a conservative region while validating inverter acceptance.
#define METASENSE_LEAF_1D4_TEMPLATE_MAX_ABS_TORQUE_NM 3.75f
#endif
#ifndef METASENSE_LEAF_1D4_TORQUE_LSB_NM
#define METASENSE_LEAF_1D4_TORQUE_LSB_NM 0.0625f
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_B0
#define METASENSE_LEAF_1D4_TEMPLATE_B0 0x6EU  // Static per Thunderstruck reference
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_B1
#define METASENSE_LEAF_1D4_TEMPLATE_B1 0x6EU  // Static per Thunderstruck reference
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_B2
#define METASENSE_LEAF_1D4_TEMPLATE_B2 0x00U  // Torque MSB
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_B3
#define METASENSE_LEAF_1D4_TEMPLATE_B3 0x00U  // Torque LSB
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_B4
#define METASENSE_LEAF_1D4_TEMPLATE_B4 0x87U  // Rolling counter (high nibble), fixed 0x7 (low nibble)
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_B5
#define METASENSE_LEAF_1D4_TEMPLATE_B5 0x44U  // Static charge status per Thunderstruck reference
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_B6
#define METASENSE_LEAF_1D4_TEMPLATE_B6 0x01U  // Static field per Thunderstruck reference
#endif
#ifndef METASENSE_LEAF_1D4_TEMPLATE_B7
#define METASENSE_LEAF_1D4_TEMPLATE_B7 0x00U  // CRC placeholder (will be overwritten)
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

#ifndef METASENSE_LEAF_1D4_RAW_SNIFF_ONLY
#define METASENSE_LEAF_1D4_RAW_SNIFF_ONLY 0
#endif

#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_REFRESH_MS
// Periodically re-issue a zero-cross pulse to refresh inverter state before timeout fallback.
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_REFRESH_MS 500U
#endif
#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_PULSE_MS
// Keep the negative pulse long enough to cover multiple 10 ms TX frames.
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_PULSE_MS 20U
#endif
#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_POS_NM
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_POS_NM 0.60f
#endif
#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_PRE_POS_MS
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_PRE_POS_MS 20U
#endif
#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_POST_POS_MS
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_POST_POS_MS 20U
#endif

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
    // Motorola: MSB at start bit 23 (byte 2 bit 7), spans byte 2 bits 7-0 + byte 3 bits 7-4
    const uint16_t raw12 = static_cast<uint16_t>(torqueRaw) & 0x0FFFU;
    frame[2] = static_cast<uint8_t>((raw12 >> 4) & 0xFFU);
    frame[3] = static_cast<uint8_t>((frame[3] & 0x0FU) | ((raw12 & 0x000FU) << 4));
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
constexpr uint8_t kKpPotPin = 6;       // swapped with massflow per latest wiring
constexpr uint8_t kMassflowPin = 8;    // kept off GPIO5 to avoid CAN RX conflict
constexpr uint8_t kLambdaPin = 7;      // ADC1_CH6
constexpr uint8_t kLoadCellPin = 32;   // Load-cell analog input

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
           MetaSense::HardwareOutputStateMachine::isFaultState();
}

static uint32_t s_initCanTxLastMs = 0U;

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
        MetaSense::Input::updateCanRpm(leafFb.rpm);
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

    // Simulate Leaf feedback path on bench setups without inverter on the CAN
    // bus. No tachometer input anymore (CAN is the sole RPM source) -- this
    // sim path (disabled by default) now injects a fixed placeholder RPM.
    const float emotorRpm = 0.0f;

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
    // Browser expects "data" message type with core fields
    // When METASENSE_CAN_MONITOR_MODE=0 (production): minimal JSON, reduced CPU/WiFi load
    // When METASENSE_CAN_MONITOR_MODE=1 (debug): full metrics including leaf_1da/1d4/11a CAN frame data
    if (now - lastDashboardMs >= dashboardCadenceMs) {
        lastDashboardMs = now;
        
        const auto& leafFb = MetaSense::CANBus::feedback();

#if METASENSE_CAN_MONITOR_MODE == 0
        // MODE 0 (PRODUCTION): Minimal JSON without CAN metrics for CPU efficiency
        // Reduced buffer size for production telemetry
        static char jsonBuffer[1200];  // Smaller buffer: production mode
        int pos = 0;
        
        // Build minimal JSON using snprintf
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
            "\"leaf_1da_inv_fault_map\":%d,"
            "\"leaf_1da_inv_status_bit\":%d,"
            "\"leaf_1da_inv_temp\":%.1f,"
            "\"leaf_1da_stator_temp\":%.1f,"
            "\"leaf_coolant_temp\":%.1f,"
            "\"leaf_ready\":%d,"
            "\"hw_precharge\":%d,"
            "\"hw_rb_plus\":%d,"
            "\"hw_rb_minus\":%d,"
            "\"hw_ssr\":%d,"
            "\"hw_state\":\"%s\""
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
            leafFb.input_voltage,  // leaf_1da_input_v
            (int)leafFb.inv_fault_map,  // leaf_1da_inv_fault_map
            (int)leafFb.inv_status_bit,  // leaf_1da_inv_status_bit
            data.leaf_invTempC,  // leaf_1da_inv_temp
            data.leaf_statorTempC,  // leaf_1da_stator_temp
            data.leaf_coolantTempC,  // leaf_coolant_temp
            leafFb.ready ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::isPrechargeActive() ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::isRbPlusActive() ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::isRbMinusActive() ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::isSsrActive() ? 1 : 0,
            MetaSense::HardwareOutputStateMachine::stateName()
        );

#else  // METASENSE_CAN_MONITOR_MODE == 1
        // MODE 1 (DEBUG): Full JSON WITH all CAN metrics for monitoring/debugging
        // Use static buffer for JSON to avoid String() float conversion issues
        static char jsonBuffer[2600];  // Larger buffer: debug mode with all metrics
        int pos = 0;
        
        // FIX: Use stats data that was captured ATOMICALLY at frame reception
        // last1daData[] and last1daWireCrcCalc are always synchronized (set together in CANBus handler)
        // This guarantees CRC RX, Calc, and OK are from the SAME frame reception event
        const auto& canStats = MetaSense::CANBus::stats();
        const uint8_t leaf1daCrcRx = canStats.last1daData[7];  // CRC RX from last reception
        const uint8_t leaf1daCrcCalc = canStats.last1daWireCrcCalc;  // CRC Calc from same reception
        const int leaf1daCrcOk = (canStats.last1daWireCrcOk > 0) ? 1 : 0;  // Match result from same reception
        
        // Build JSON with full CAN metrics using snprintf for robust numeric formatting
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
            "\"hw_state\":\"%s\""
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
            MetaSense::HardwareOutputStateMachine::stateName()  // hw_state (INIT/START/IDLE/MOTOR/DYNO)
        );
#endif  // METASENSE_CAN_MONITOR_MODE
        
        wsock.textAll(jsonBuffer);
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

void setLeaf11aUiGear(uint8_t gear)
{
    s_leaf11aUiGear = static_cast<uint8_t>(gear & 0x0FU);
    // Update all 4 template slots with the new gear value
    for (uint8_t muxSel = 0U; muxSel < 4U; ++muxSel) {
        uint8_t* slot = s_leaf11aMuxTemplateBySel[muxSel];
        // Byte 0: bits 4-7 contain the gear value
        slot[0] = static_cast<uint8_t>((slot[0] & 0x0FU) | ((s_leaf11aUiGear & 0x0FU) << 4U));
    }
}

uint8_t getLeaf11aUiGear()
{
    return s_leaf11aUiGear;
}

void setLeaf11aUiCarOnOff(uint8_t carOnOff)
{
    s_leaf11aUiCarOnOff = static_cast<uint8_t>(carOnOff & 0x07U);
    // Update all 4 template slots with the new car on/off value
    for (uint8_t muxSel = 0U; muxSel < 4U; ++muxSel) {
        uint8_t* slot = s_leaf11aMuxTemplateBySel[muxSel];
        // Byte 1: bits 5-7 contain the car on/off value
        slot[1] = static_cast<uint8_t>((slot[1] & 0x1FU) | ((s_leaf11aUiCarOnOff & 0x07U) << 5U));
    }
}

uint8_t getLeaf11aUiCarOnOff()
{
    return s_leaf11aUiCarOnOff;
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

    if (s_leaf11aMuxSeenMask == 0U) {
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
    // Note: Gear, CarOnOff, and Eco values come from the template which is updated by UI controls
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

    // Bootstrap seeds all 4 mux template slots with fixed defaults from CanConfig.h.
    // (Previously these could be refined by learning from received TVCU 0x11A
    // frames, but 0x11A is no longer a processed RX frame, so templates are
    // fixed at bootstrap.)
    s_leaf11aMuxSeenMask = 0x0FU;
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
    patchLeaf1d4TorqueFieldMotorola23_12(data, fields.torqueDemandRaw);
    setIntelUnsigned(data, 8U, 34U, 1U, fields.hvStatus ? 1U : 0U);
    setIntelUnsigned(data, 8U, 46U, 1U, fields.rbPlus ? 1U : 0U);
    setIntelUnsigned(data, 8U, 38U, 2U, static_cast<uint32_t>(fields.cmdClock & 0x03U));
    // Fixed template header bytes (0x1D4 ring buffer subsystem was removed - always template mode).
    data[0] = METASENSE_LEAF_1D4_TEMPLATE_B0;
    data[1] = METASENSE_LEAF_1D4_TEMPLATE_B1;
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

void logLeaf1d4ShadowFrame(uint32_t nowMs,
                           float torqueDemandNm,
                           bool readyBit,
                           bool hvOkBit,
                           bool brakeBit,
                           bool gearDriveBit,
                           bool txSent)
{
    (void)nowMs;
    (void)torqueDemandNm;
    (void)readyBit;
    (void)hvOkBit;
    (void)brakeBit;
    (void)gearDriveBit;
    (void)txSent;
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

    // Frame base always uses fixed template header bytes (0x1D4 ring buffer
    // subsystem, which sniffed a live frame as base, was removed as dead code).
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
    
    // Keep template header bytes explicit (data[] was already seeded with
    // template values above, this reasserts bytes 0-1 for clarity).
    data[0] = METASENSE_LEAF_1D4_TEMPLATE_B0;
    data[1] = METASENSE_LEAF_1D4_TEMPLATE_B1;

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
            logLeaf1d4ShadowFrame(nowMs,
                                  s_leaf1d4PayloadTorqueNm,
                                  s_leafTxPacerReadyBit,
                                  s_leafTxPacerHvOkBit,
                                  s_leafTxPacerBrakeBit,
                                  s_leafTxPacerGearDriveBit,
                                  sent1d4);
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

    // CAN bus init: configure + let the control-loop poll() start the TWAI
    // driver, then start the TX pacer task. 0x1D4 (torque demand) and 0x11A
    // (keep-alive) begin transmitting every 10ms with template payloads
    // (torque demand=0, gear=4, car=2, charge status=1, manual mode) as soon
    // as the driver comes up. It is not mandatory for CAN to be the first
    // thing initialized at boot, but the VCU state machine (see
    // HardwareOutputStateMachine.cpp) requires live 0x1DA feedback freshness
    // before it will leave INIT and start the precharge sequence.
    MetaSense::CANBus::configure(kLeafCanConfig);

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
        logLeaf1d4ShadowFrame(millis(),
                              0.0f,
                              false,
                              false,
                              false,
                              false,
                              sent);
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

    // RPM source: CAN 0x1DA e-motor RPM is the sole RPM source. Test-engine
    // RPM = e-motor RPM * gear ratio. No tachometer fallback -- the whole
    // control chain (VCU state machine, torque commands) already requires
    // live CAN feedback to do anything, so a fallback RPM source has no
    // practical meaning and risked showing a misleading "still running"
    // reading while the system is actually inoperative.
    const MetaSense::CANBus::Stats& canStatsRpm = MetaSense::CANBus::stats();
    const bool canRpmFrameFresh = (canStatsRpm.last1daMs != 0U) &&
                                  (elapsedMsSafe(now, canStatsRpm.last1daMs) <= CAN_RX_TARGET_MAX_AGE_MS);
    const bool canRpmFrameWireCrcOk = is1daWireCrcTrustedForFallback(canStatsRpm, now);
    const bool canRpmFrameTrustworthy = canRpmFrameFresh && canRpmFrameWireCrcOk;
    const bool canValid = MetaSense::CANBus::isReady() && canRpmFrameTrustworthy;
    const float emotorRpmRaw = canRpmFrameTrustworthy ? leafCanRpmMonitor : canRpm;
    const float rpmRatio = (MetaSense::Settings::virtGearRatio > 0.01f)
        ? MetaSense::Settings::virtGearRatio
        : 1.45f;
    const float canEngineRpm = emotorRpmRaw * rpmRatio;
    if (canValid) {
        // Mandatory behavior: CAN source is unfiltered.
        rpmFilt = canEngineRpm;
    } else {
        rpmFilt = lpFilter(rpmFilt, 0.0f, alpha);
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

    // other sensors
    float drumRaw = canValid ? emotorRpmRaw : 0.0f;
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

    // VCU state machine inputs: RPM/HV/fault come exclusively from the
    // decoded 0x1DA inverter feedback frame -- no tachometer fallback, no
    // bench-forced override. Freshness of leafFb.rpm_update_ms is the only
    // "live CAN feedback" signal the state machine trusts.
    const LeafInvFeedback& leafFb = MetaSense::CANBus::feedback();
    const bool canFeedbackFreshForHwsm = (leafFb.rpm_update_ms != 0U) &&
        (elapsedMsSafe(now, leafFb.rpm_update_ms) <= CAN_RX_TARGET_MAX_AGE_MS);
    const bool inverterFaultFromCan = canFeedbackFreshForHwsm && (leafFb.mg_error_codes != 0U);

    MetaSense::HardwareOutputStateMachine::update(
        engineThrottlePercent,
        tele.rpmTarget,
        primaryBrakeSignedPercent,
        leafFb.rpm,
        leafFb.input_voltage,
        inverterFaultFromCan,
        canFeedbackFreshForHwsm);

    const bool ssrActiveForLeafTx = MetaSense::HardwareOutputStateMachine::isSsrActive();
    const bool prechargeActiveForLeafTx = MetaSense::HardwareOutputStateMachine::isPrechargeActive();
    const bool prechargeSucceededForLeafTx = MetaSense::HardwareOutputStateMachine::isPrechargeSucceeded();
    const LeafInvFeedback& leafFbDiag = MetaSense::CANBus::feedback();

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
            logLeaf1d4ShadowFrame(now,
                                  previewTorqueNm,
                                  inverterReadyPreview,
                                  hvOkPreview,
                                  brakePreview,
                                  gearDrivePreview,
                                  sentManual);
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
                logLeaf1d4ShadowFrame(now, 0.0f, false, false, false, true, sent);
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
        const bool isIdleState = MetaSense::HardwareOutputStateMachine::isIdleState();
        
        // Safety: Auto-reset manual torque when entering INIT state
        if (isInitState && s_lastHardwareState != hwState) {
            setLeafUiTorqueDemandNmInternal(0.0f);
            Serial.println("[SAFETY] Manual torque reset to 0.0 Nm - entering INIT state");
        }
        s_lastHardwareState = hwState;
        
        if (isInitState) {
            // INIT state: force torque demand to 0 Nm during startup (safety critical)
            torqueToSend = 0.0f;
        } else if (isIdleState) {
            // IDLE state: Apply voltage-based brake torque controller for inverter overvoltage protection
            // When inverter voltage > 450V, increase brake torque to dissipate more power (V × I)
            const float invVoltageV = MetaSense::CANBus::feedback().input_voltage;
            const float appliedBrakeTorque = computeVoltageBrakeTorque(
                invVoltageV,                               // Inverter voltage from CAN
                MetaSense::Settings::idleTorqueNm,         // Base idle torque (4Nm default)
                MetaSense::Settings::brakeMaxTorqueNm      // Max brake torque (30Nm default)
            );
            torqueToSend = appliedBrakeTorque;
        } else {
            // MOTOR state: apply manual/auto torque selection
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

        // SAFETY: Reverse-rotation guard. Applies unconditionally, after all
        // state-specific torque selection above, regardless of which branch
        // (INIT/IDLE/MOTOR) produced torqueToSend. Below 100 RPM, force the
        // 0x1D4 torque demand payload to exactly 0 Nm (any sign) so the
        // motor can never be driven into reverse rotation from near
        // standstill -- this would not be tolerated in a real EV and must
        // not be tolerated here either.
        constexpr float kReverseGuardRpmThreshold = 100.0f;
        if (tele.rpm < kReverseGuardRpmThreshold) {
            torqueToSend = 0.0f;
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
        const char* vcuStateName = MetaSense::HardwareOutputStateMachine::stateName();
        const auto& fbForHb = MetaSense::CANBus::feedback();
        Serial.printf("[HEARTBEAT] RPM=%.0f Leaf_RPM=%.0f HV=%.1f VCU=%s mg_err=0x%02X inv_fault=0x%02X inv_status=%d Temps: Inv=%.1f Stator=%.1f Coolant=%.1f IP=%d.%d.%d.%d RSSI=%ld dBm ms=%lu\n",
                      tele.rpm, tele.leaf_rpm, tele.vcuHvVoltage, vcuStateName,
                      (unsigned)fbForHb.mg_error_codes, (unsigned)fbForHb.inv_fault_map, (int)fbForHb.inv_status_bit,
                      tele.leaf_invTempC, tele.leaf_statorTempC, tele.leaf_coolantTempC,
                      ip[0], ip[1], ip[2], ip[3], rssi, nowMs);
        // Forward to telnet clients
        MetaSense::TelnetSerialBridge::telnetBridgePrintf("[HEARTBEAT] RPM=%.0f Leaf_RPM=%.0f HV=%.1f VCU=%s mg_err=0x%02X inv_fault=0x%02X inv_status=%d Temps: Inv=%.1f Stator=%.1f Coolant=%.1f IP=%d.%d.%d.%d RSSI=%ld dBm ms=%lu\n",
                      tele.rpm, tele.leaf_rpm, tele.vcuHvVoltage, vcuStateName,
                      (unsigned)fbForHb.mg_error_codes, (unsigned)fbForHb.inv_fault_map, (int)fbForHb.inv_status_bit,
                      tele.leaf_invTempC, tele.leaf_statorTempC, tele.leaf_coolantTempC,
                      ip[0], ip[1], ip[2], ip[3], rssi, nowMs);

        // CAN bus health, visible over telnet (driver ready state + RX frame
        // counts + TX pacer loop iterations + TWAI controller health, since
        // there is no longer a dedicated TX-sent counter after the metrics
        // cleanup). twai_tx_q/tx_err/bus_err help distinguish "not
        // transmitting at all" from "transmitting but never ACKed".
        const MetaSense::CANBus::Stats& canStatsHb = MetaSense::CANBus::stats();
        MetaSense::TelnetSerialBridge::telnetBridgePrintf(
            "[CAN-STATUS] ready=%d rx_1da=%lu rx_55a=%lu bus_off=%lu status_fail=%lu tx_loops=%lu twai(state=%u tx_q=%lu rx_q=%lu tx_err=%lu rx_err=%lu bus_err=%lu arb_lost=%lu)\n",
            canStatsHb.ready ? 1 : 0,
            static_cast<unsigned long>(canStatsHb.rx1daFrames),
            static_cast<unsigned long>(canStatsHb.rx55aFrames),
            static_cast<unsigned long>(canStatsHb.busOffEvents),
            static_cast<unsigned long>(canStatsHb.statusQueryFailures),
            static_cast<unsigned long>(s_leaf1d4TxFrameCount),
            static_cast<unsigned>(canStatsHb.lastTwaiState),
            static_cast<unsigned long>(canStatsHb.twaiTxQueued),
            static_cast<unsigned long>(canStatsHb.twaiRxQueued),
            static_cast<unsigned long>(canStatsHb.twaiTxErrorCounter),
            static_cast<unsigned long>(canStatsHb.twaiRxErrorCounter),
            static_cast<unsigned long>(canStatsHb.twaiBusError),
            static_cast<unsigned long>(canStatsHb.twaiArbLost));
        
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
