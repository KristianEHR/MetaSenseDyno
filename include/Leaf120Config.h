#ifndef METASENSE_LEAF_120_CONFIG_H
#define METASENSE_LEAF_120_CONFIG_H

#include <cstdint>

// Leaf 0x120 Frame Configuration (SHADOW TRACKING ONLY)
// =====================================================
// PRODUCTION STATUS: 0x120 TX code removed - using 0x1D4 for active torque control.
// 
// This header provides configuration for 0x120 SHADOW STATE TRACKING ONLY.
// Shadow state monitoring is used for diagnostics/telemetry visibility.
// All active torque transmission uses 0x1D4 (sendLeafTorqueCommand1d4).
//
// Settings control shadow state machine, noise filtering, and diagnostic logging.

// Core 0x120 Logging
// ================================
#ifndef METASENSE_LEAF_120_AUX_LOGS
#define METASENSE_LEAF_120_AUX_LOGS 0
#endif

#ifndef METASENSE_LEAF_120_SHADOW_LOGS
#define METASENSE_LEAF_120_SHADOW_LOGS 1
#endif

#ifndef METASENSE_LEAF_120_COMPARE_COPY_RX
#define METASENSE_LEAF_120_COMPARE_COPY_RX 0
#endif

// Shadow State Strategy
// ================================
// 0: use explicit brake/gear inputs from control path
// 1: derive state from torque sign (+ = MOTOR, - = BRAKE)
// 2: 4-quadrant model using torque sign and RPM sign
//    (0Nm=Neutral, +Nm=Forward, -Nm with rpm>0=Brake, -Nm with rpm<0=Reverse)
#ifndef METASENSE_LEAF_120_SHADOW_STATE_STRATEGY
#define METASENSE_LEAF_120_SHADOW_STATE_STRATEGY 0
#endif

// Shadow State Deadband
// Torque deadband for state transitions (Nm)
#ifndef METASENSE_LEAF_120_SHADOW_TORQUE_DEADBAND_NM
#define METASENSE_LEAF_120_SHADOW_TORQUE_DEADBAND_NM 1.0f
#endif

// Shadow State Startup Behavior
// Milliseconds: >0 keeps state in neutral for startup transient window before forcing F states
#ifndef METASENSE_LEAF_120_SHADOW_NEUTRAL_STARTUP_MS
#define METASENSE_LEAF_120_SHADOW_NEUTRAL_STARTUP_MS 0U
#endif

// Shadow State Idle Behavior
// In torque-sign strategy, keep F in near-zero torque zone when enabled
#ifndef METASENSE_LEAF_120_SHADOW_IDLE_FORCE_FWD
#define METASENSE_LEAF_120_SHADOW_IDLE_FORCE_FWD 1
#endif

// Shadow State Reverse Detection
// RPM threshold for detecting reverse motion (abs RPM)
#ifndef METASENSE_LEAF_120_SHADOW_REVERSE_RPM_DEADBAND
#define METASENSE_LEAF_120_SHADOW_REVERSE_RPM_DEADBAND 30.0f
#endif

// Facts Logging (0x120 Event Tracking)
// ================================
#ifndef METASENSE_LEAF_120_FACTS_LOGS
#define METASENSE_LEAF_120_FACTS_LOGS 0
#endif

// Facts log period (milliseconds)
#ifndef METASENSE_LEAF_120_FACTS_LOG_PERIOD_MS
#define METASENSE_LEAF_120_FACTS_LOG_PERIOD_MS 5000U
#endif

// Noise Filtering and Analysis
// ================================
#ifndef METASENSE_LEAF_120_NOISE_LOGS
#define METASENSE_LEAF_120_NOISE_LOGS 0
#endif

// Noise detection threshold (Nm)
// Values within ±this range around filtered value are considered noise
#ifndef METASENSE_LEAF_120_NOISE_DB_NM
#define METASENSE_LEAF_120_NOISE_DB_NM 0.4f
#endif

// EMA (Exponential Moving Average) alpha for noise filtering
// Higher value = faster response to changes, more susceptible to noise
#ifndef METASENSE_LEAF_120_NOISE_EMA_ALPHA
#define METASENSE_LEAF_120_NOISE_EMA_ALPHA 0.10f
#endif

// Outlier rejection threshold (Nm)
// Jumps larger than this are rejected as measurement errors
#ifndef METASENSE_LEAF_120_NOISE_OUTLIER_ABS_NM
#define METASENSE_LEAF_120_NOISE_OUTLIER_ABS_NM 5.0f
#endif

// Rate-of-change filtering for torque dynamics
// ================================
// Steady state slope (Nm/s) - for gentle ramps
#ifndef METASENSE_LEAF_120_DT_SLOPE_STEADY_NMPS
#define METASENSE_LEAF_120_DT_SLOPE_STEADY_NMPS 10.0f
#endif

// High dynamic slope (Nm/s) - for aggressive changes
#ifndef METASENSE_LEAF_120_DT_SLOPE_HIGH_NMPS
#define METASENSE_LEAF_120_DT_SLOPE_HIGH_NMPS 120.0f
#endif

// 0x120 Transmission Control
// ================================
#ifndef METASENSE_LEAF_120_TX_COMMIT_ENABLED
#define METASENSE_LEAF_120_TX_COMMIT_ENABLED 0
#endif

// 0x120 Startup Behavior
// Hold 0x120 torque demand at zero during initial HV bring-up so the inverter
// can precharge cleanly and clear its error bits before any torque is applied
#ifndef METASENSE_LEAF_120_STARTUP_ZERO_TORQUE
#define METASENSE_LEAF_120_STARTUP_ZERO_TORQUE 1
#endif

#endif // METASENSE_LEAF_120_CONFIG_H
