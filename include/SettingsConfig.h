/**
 * @file SettingsConfig.h
 * @brief Centralized Settings module configuration parameters
 * 
 * Consolidates all Settings-related configuration defaults, Preferences keys,
 * LittleFS paths, and system limits extracted from:
 * - src/Settings.cpp/h (defaults, Preferences storage)
 * - src/CommandRouter.cpp (run/profile management)
 * - src/HardwareOutputStateMachine.cpp (relay & state thresholds)
 * - include/globals.h (PWM configuration)
 * - src/Input.cpp (CAN/Leaf timeouts, load cell calibration)
 * 
 * Phase 14B: Parameter consolidation for unified configuration management.
 */

#pragma once

// ============================================================================
// TORQUE CONTROL: Limits, Idle, Brake Max
// ============================================================================

// Idle torque demand (active in idle mode, range 0.0-10.0 Nm)
#ifndef METASENSE_SETTINGS_IDLE_TORQUE_NM
#define METASENSE_SETTINGS_IDLE_TORQUE_NM 4.0f
#endif

// Brake maximum torque (range 0.0-200.0 Nm, for load cell dyno mode)
#ifndef METASENSE_SETTINGS_BRAKE_MAX_TORQUE_NM
#define METASENSE_SETTINGS_BRAKE_MAX_TORQUE_NM 30.0f
#endif

// Torque track step size for convergence detection
#ifndef METASENSE_TORQUE_TRACK_STEP_NM
#define METASENSE_TORQUE_TRACK_STEP_NM 2.0f
#endif

// Torque track absolute tolerance (stop when |target - actual| < this)
#ifndef METASENSE_TORQUE_TRACK_ABS_TOL_NM
#define METASENSE_TORQUE_TRACK_ABS_TOL_NM 3.0f
#endif

// Torque track relative tolerance (stop when error_percent < this)
#ifndef METASENSE_TORQUE_TRACK_REL_TOL
#define METASENSE_TORQUE_TRACK_REL_TOL 0.10f
#endif

// Torque track convergence timeout
#ifndef METASENSE_LEAF_TORQUE_TRACK_TIMEOUT_MS
#define METASENSE_LEAF_TORQUE_TRACK_TIMEOUT_MS 3000
#endif

// ============================================================================
// RPM CONTROL: Limits, Setpoints, Modes
// ============================================================================

// Motor mode maximum RPM (default target)
#ifndef METASENSE_SETTINGS_MOTOR_MODE_MAX_RPM
#define METASENSE_SETTINGS_MOTOR_MODE_MAX_RPM 2000.0f
#endif

// Default RPM target for run profiles
#ifndef METASENSE_SETTINGS_RPM_TARGET
#define METASENSE_SETTINGS_RPM_TARGET 0.0f
#endif

// Default RPM start (for ramp profiles)
#ifndef METASENSE_SETTINGS_RPM_START
#define METASENSE_SETTINGS_RPM_START 1500.0f
#endif

// Default RPM end (for ramp profiles)
#ifndef METASENSE_SETTINGS_RPM_END
#define METASENSE_SETTINGS_RPM_END 5500.0f
#endif

// Tachometer calibration factor (pulses per RPM or scaling)
#ifndef METASENSE_SETTINGS_TACHO_CAL
#define METASENSE_SETTINGS_TACHO_CAL 10.0f
#endif

// Maximum RPM (gauge display limit, safety limit)
#ifndef METASENSE_SETTINGS_MAX_RPM
#define METASENSE_SETTINGS_MAX_RPM 18000.0f
#endif

// RPM hysteresis band (prevent chatter near state transitions)
#ifndef METASENSE_RPM_HYSTERESIS_BAND_RPM
#define METASENSE_RPM_HYSTERESIS_BAND_RPM 100.0f
#endif

// RPM threshold for dyno mode entry
#ifndef METASENSE_DYNO_MODE_ENTRY_RPM
#define METASENSE_DYNO_MODE_ENTRY_RPM 500.0f
#endif

// RPM threshold for idle mode entry
#ifndef METASENSE_IDLE_MODE_ENTRY_MAX_RPM
#define METASENSE_IDLE_MODE_ENTRY_MAX_RPM 500.0f
#endif

// RPM threshold for idle setpoint (zero torque activation)
#ifndef METASENSE_IDLE_SETPOINT_ZERO_THRESHOLD_RPM
#define METASENSE_IDLE_SETPOINT_ZERO_THRESHOLD_RPM 0.0f
#endif

// ============================================================================
// VOLTAGE THRESHOLDS & LIMITS: HV Relay, VCM Ready, Battery
// ============================================================================

// HV relay high voltage threshold (turn on relay)
#ifndef METASENSE_HV_RELAY_HIGH_V
#define METASENSE_HV_RELAY_HIGH_V 320.0f
#endif

// HV relay low voltage threshold (turn off relay)
#ifndef METASENSE_HV_RELAY_LOW_V
#define METASENSE_HV_RELAY_LOW_V 300.0f
#endif

// VCM HV ready voltage threshold (precharge complete)
#ifndef METASENSE_LEAF_VCM_HV_READY_VOLTAGE_V
#define METASENSE_LEAF_VCM_HV_READY_VOLTAGE_V 300.0f
#endif

// ============================================================================
// HARDWARE OUTPUT STATE MACHINE: Relay Timing & State Transitions
// ============================================================================

// Relay switching delay (ms) - debounce contact bounce
#ifndef METASENSE_RELAY_SWITCH_DELAY_MS
#define METASENSE_RELAY_SWITCH_DELAY_MS 20
#endif

// State debounce time (ms) - wait before accepting state change
#ifndef METASENSE_STATE_DEBOUNCE_MS
#define METASENSE_STATE_DEBOUNCE_MS 50
#endif

// Initial prerequisites stable time (ms) - before precharge sequence
#ifndef METASENSE_INIT_PREREQ_STABLE_MS
#define METASENSE_INIT_PREREQ_STABLE_MS 0
#endif

// Precharge minimum duration (ms) - allow capacitors to charge
#ifndef METASENSE_INIT_PRECHARGE_MIN_MS
#define METASENSE_INIT_PRECHARGE_MIN_MS 2500
#endif

// Start setpoint wait time (ms) - after ready, before allowing torque
#ifndef METASENSE_START_SETPOINT_WAIT_MS
#define METASENSE_START_SETPOINT_WAIT_MS 15000
#endif

// Relay invariant state log period (ms)
#ifndef METASENSE_RELAY_INVARIANT_LOG_PERIOD_MS
#define METASENSE_RELAY_INVARIANT_LOG_PERIOD_MS 1000
#endif

// ============================================================================
// LOAD CELL & SENSOR CALIBRATION
// ============================================================================

// Load cell raw scale (ADC counts to physical units)
#ifndef METASENSE_LOAD_CELL_RAW_SCALE
#define METASENSE_LOAD_CELL_RAW_SCALE 0.01f
#endif

// Load cell calibration factor (kg per ADC count)
#ifndef METASENSE_LOAD_CELL_CALIBRATION_FACTOR
#define METASENSE_LOAD_CELL_CALIBRATION_FACTOR 0.01f
#endif

// Default load cell gain (NAU7802_GAIN_* value, typically 128)
#ifndef METASENSE_LOAD_CELL_DEFAULT_GAIN
#define METASENSE_LOAD_CELL_DEFAULT_GAIN 128
#endif

// Default load cell sample rate (SPS - typically 320)
#ifndef METASENSE_LOAD_CELL_DEFAULT_RATE_SPS
#define METASENSE_LOAD_CELL_DEFAULT_RATE_SPS 320
#endif

// Known weight for calibration procedure (kg)
#ifndef METASENSE_LOAD_CELL_CALIBRATION_KNOWN_WEIGHT_KG
#define METASENSE_LOAD_CELL_CALIBRATION_KNOWN_WEIGHT_KG 3.404f
#endif

// ============================================================================
// FILTER & AVERAGING PARAMETERS
// ============================================================================

// Default low-pass filter alpha (0.0-1.0, higher = less filtering)
#ifndef METASENSE_SETTINGS_FILTER_ALPHA
#define METASENSE_SETTINGS_FILTER_ALPHA 0.2f
#endif

// Load cell averaging window size (first stage)
#ifndef METASENSE_SETTINGS_LOAD_AVG_N
#define METASENSE_SETTINGS_LOAD_AVG_N 5.0f
#endif

// Load cell averaging window size (second stage, two-stage mode)
#ifndef METASENSE_SETTINGS_LOAD_AVG_N2
#define METASENSE_SETTINGS_LOAD_AVG_N2 5.0f
#endif

// Load cell filter mode (0=moving average, 1=two-stage)
#ifndef METASENSE_SETTINGS_LOAD_FILTER_MODE
#define METASENSE_SETTINGS_LOAD_FILTER_MODE 1
#endif

// Load raw average window maximum size
#ifndef METASENSE_LOAD_RAW_AVERAGE_WINDOW_MAX
#define METASENSE_LOAD_RAW_AVERAGE_WINDOW_MAX 255
#endif

// Load raw average window default size
#ifndef METASENSE_LOAD_RAW_AVERAGE_WINDOW_DEFAULT
#define METASENSE_LOAD_RAW_AVERAGE_WINDOW_DEFAULT 5
#endif

// Tachometer raw average window size
#ifndef METASENSE_TACHO_RAW_AVERAGE_WINDOW
#define METASENSE_TACHO_RAW_AVERAGE_WINDOW 5
#endif

// Auxiliary analog average window size (mass flow, lambda, etc.)
#ifndef METASENSE_AUX_RAW_AVERAGE_WINDOW
#define METASENSE_AUX_RAW_AVERAGE_WINDOW 8
#endif

// ============================================================================
// PI CONTROLLER TUNING PARAMETERS
// ============================================================================

// Default proportional gain (Kp)
#ifndef METASENSE_SETTINGS_DEFAULT_KP
#define METASENSE_SETTINGS_DEFAULT_KP 0.073f
#endif

// Default integral gain (Ki)
#ifndef METASENSE_SETTINGS_DEFAULT_KI
#define METASENSE_SETTINGS_DEFAULT_KI 0.524f
#endif

// Legacy Kp (older tuning)
#ifndef METASENSE_SETTINGS_LEGACY_DEFAULT_KP
#define METASENSE_SETTINGS_LEGACY_DEFAULT_KP 0.02f
#endif

// Legacy Ki (older tuning)
#ifndef METASENSE_SETTINGS_LEGACY_DEFAULT_KI
#define METASENSE_SETTINGS_LEGACY_DEFAULT_KI 0.05f
#endif

// Use POT3 for runtime Kp adjustment
#ifndef METASENSE_SETTINGS_USE_POT3_KP
#define METASENSE_SETTINGS_USE_POT3_KP false
#endif

// Runtime Kp adaptation minimum
#ifndef METASENSE_RUNTIME_KP_MIN
#define METASENSE_RUNTIME_KP_MIN 0.005f
#endif

// Runtime Kp adaptation maximum
#ifndef METASENSE_RUNTIME_KP_MAX
#define METASENSE_RUNTIME_KP_MAX 0.200f
#endif

// Runtime Kp adaptation alpha (EMA smoothing)
#ifndef METASENSE_RUNTIME_KP_ALPHA
#define METASENSE_RUNTIME_KP_ALPHA 0.12f
#endif

// Runtime Kp adaptation step size
#ifndef METASENSE_RUNTIME_KP_APPLY_DELTA
#define METASENSE_RUNTIME_KP_APPLY_DELTA 0.001f
#endif

// ============================================================================
// GAUGE DISPLAY RANGES
// ============================================================================

// Maximum HP display on gauge
#ifndef METASENSE_SETTINGS_MAX_HP
#define METASENSE_SETTINGS_MAX_HP 25.0f
#endif

// Maximum torque display on gauge
#ifndef METASENSE_SETTINGS_MAX_TORQUE
#define METASENSE_SETTINGS_MAX_TORQUE 200.0f
#endif

// Dynamometer arm length (cm) for torque calculation
#ifndef METASENSE_SETTINGS_ARM_CM
#define METASENSE_SETTINGS_ARM_CM 20.0f
#endif

// ============================================================================
// INERTIA DYNO PARAMETERS
// ============================================================================

// Drum mass (kg) for inertia calculation
#ifndef METASENSE_SETTINGS_DRUM_MASS_KG
#define METASENSE_SETTINGS_DRUM_MASS_KG 10.0f
#endif

// Drum radius (m) for rotational inertia
#ifndef METASENSE_SETTINGS_DRUM_RADIUS_M
#define METASENSE_SETTINGS_DRUM_RADIUS_M 0.15f
#endif

// Drum wall thickness (m) - if 0, assume solid cylinder
#ifndef METASENSE_SETTINGS_DRUM_WALL_M
#define METASENSE_SETTINGS_DRUM_WALL_M 0.0f
#endif

// Drivetrain efficiency (percent, 0-100)
#ifndef METASENSE_SETTINGS_DRIVETRAIN_EFF_PCT
#define METASENSE_SETTINGS_DRIVETRAIN_EFF_PCT 95.0f
#endif

// Virtual gear ratio (for multi-speed dyno simulation)
#ifndef METASENSE_SETTINGS_VIRT_GEAR_RATIO
#define METASENSE_SETTINGS_VIRT_GEAR_RATIO 1.0f
#endif

// Enable inertia dyno mode (vs. load dyno)
#ifndef METASENSE_SETTINGS_INERTIA_MODE_ENABLED
#define METASENSE_SETTINGS_INERTIA_MODE_ENABLED false
#endif

// ============================================================================
// PULSE INPUT CONFIGURATION
// ============================================================================

// Engine tacho pulses per revolution
#ifndef METASENSE_SETTINGS_PULSES_PER_REV_ENGINE
#define METASENSE_SETTINGS_PULSES_PER_REV_ENGINE 1.0f
#endif

// Drum tacho pulses per revolution
#ifndef METASENSE_SETTINGS_PULSES_PER_REV_DRUM
#define METASENSE_SETTINGS_PULSES_PER_REV_DRUM 1.0f
#endif

// ============================================================================
// SETTINGS STORAGE: Preferences Namespace & LittleFS Paths
// ============================================================================

// Preferences namespace for all Settings data
#ifndef METASENSE_SETTINGS_PREFS_NAMESPACE
#define METASENSE_SETTINGS_PREFS_NAMESPACE "settings"
#endif

// Preferences key for settings initialization marker
#ifndef METASENSE_SETTINGS_PREFS_INIT_KEY
#define METASENSE_SETTINGS_PREFS_INIT_KEY "init"
#endif

// LittleFS path for factory default profile
#ifndef METASENSE_FACTORY_PROFILE_PATH
#define METASENSE_FACTORY_PROFILE_PATH "/factory_profile.json"
#endif

// ============================================================================
// PREFERENCES KEYS (as stored in NVRAM)
// ============================================================================

#define METASENSE_PREFS_KEY_FILTER_ALPHA "filterAlpha"
#define METASENSE_PREFS_KEY_LOAD_AVG_N "loadAvgN"
#define METASENSE_PREFS_KEY_LOAD_AVG_N2 "loadAvgN2"
#define METASENSE_PREFS_KEY_LC_FILTER_MODE "lcFMode"
#define METASENSE_PREFS_KEY_LC_GAIN "lcGain"
#define METASENSE_PREFS_KEY_LC_RATE "lcRate"
#define METASENSE_PREFS_KEY_KP "kp"
#define METASENSE_PREFS_KEY_KI "ki"
#define METASENSE_PREFS_KEY_POT3_KP "pot3kp"
#define METASENSE_PREFS_KEY_RH_OFFSET_PCT "rhOffPct"
#define METASENSE_PREFS_KEY_MOTOR_MAX_RPM "motorMaxRpm"
#define METASENSE_PREFS_KEY_IDLE_TORQUE "idleTorque"
#define METASENSE_PREFS_KEY_BRAKE_MAX_TQ "brakeMaxTq"
#define METASENSE_PREFS_KEY_LEAF_SIM_FB "leafSimFb"
#define METASENSE_PREFS_KEY_MAX_RPM "maxRPM"
#define METASENSE_PREFS_KEY_MAX_HP "maxHP"
#define METASENSE_PREFS_KEY_MAX_TORQUE "maxTorque"
#define METASENSE_PREFS_KEY_ARM_CM "armCm"
#define METASENSE_PREFS_KEY_PPR_ENG "pprEng"
#define METASENSE_PREFS_KEY_PPR_DRUM "pprDrum"
#define METASENSE_PREFS_KEY_DRIVE_EFF "driveEff"
#define METASENSE_PREFS_KEY_INERTIA "inertia"
#define METASENSE_PREFS_KEY_CAN_RPM "canRpm"
#define METASENSE_PREFS_KEY_DRUM_MASS "drumMass"
#define METASENSE_PREFS_KEY_DRUM_RAD "drumRad"
#define METASENSE_PREFS_KEY_DRUM_WALL "drumWall"
#define METASENSE_PREFS_KEY_GEAR_RATIO "gearRatio"
#define METASENSE_PREFS_KEY_RPM_TARGET "rpmTarget"
#define METASENSE_PREFS_KEY_TACHO_CAL "tachoCal"
#define METASENSE_PREFS_KEY_RPM_START "rpmStart"
#define METASENSE_PREFS_KEY_RPM_END "rpmEnd"

// ============================================================================
// PWM CONFIGURATION: Throttle, Servo, Actuator
// ============================================================================

// Throttle PWM frequency (Hz)
#ifndef METASENSE_THROTTLE_PWM_FREQUENCY_HZ
#define METASENSE_THROTTLE_PWM_FREQUENCY_HZ 250
#endif

// Throttle PWM resolution (bits)
#ifndef METASENSE_THROTTLE_PWM_RESOLUTION_BITS
#define METASENSE_THROTTLE_PWM_RESOLUTION_BITS 12
#endif

// Throttle PWM minimum percent (0-100%)
#ifndef METASENSE_THROTTLE_PWM_MIN_PCT
#define METASENSE_THROTTLE_PWM_MIN_PCT 0.0f
#endif

// Throttle PWM maximum percent (0-100%)
#ifndef METASENSE_THROTTLE_PWM_MAX_PCT
#define METASENSE_THROTTLE_PWM_MAX_PCT 100.0f
#endif

// Servo PWM frequency (Hz, typically 50 for standard servos)
#ifndef METASENSE_SERVO_PWM_FREQUENCY_HZ
#define METASENSE_SERVO_PWM_FREQUENCY_HZ 50
#endif

// Servo PWM resolution (bits)
#ifndef METASENSE_SERVO_PWM_RESOLUTION_BITS
#define METASENSE_SERVO_PWM_RESOLUTION_BITS 14
#endif

// Servo PWM pulse minimum (µs, typically 1000-1500)
#ifndef METASENSE_SERVO_PWM_PULSE_MIN_US
#define METASENSE_SERVO_PWM_PULSE_MIN_US 1000
#endif

// Servo PWM pulse maximum (µs, typically 1500-2000)
#ifndef METASENSE_SERVO_PWM_PULSE_MAX_US
#define METASENSE_SERVO_PWM_PULSE_MAX_US 2000
#endif

// Actuator PWM frequency (Hz, higher for faster response)
#ifndef METASENSE_ACTUATOR_PWM_FREQUENCY_HZ
#define METASENSE_ACTUATOR_PWM_FREQUENCY_HZ 5000
#endif

// Actuator PWM resolution (bits)
#ifndef METASENSE_ACTUATOR_PWM_RESOLUTION_BITS
#define METASENSE_ACTUATOR_PWM_RESOLUTION_BITS 10
#endif

// ============================================================================
// CAN/LEAF COMMUNICATION TIMEOUTS
// ============================================================================

// CAN online detection timeout (ms)
#ifndef METASENSE_LEAF_VCM_CAN_ONLINE_DETECT_MS
#define METASENSE_LEAF_VCM_CAN_ONLINE_DETECT_MS 500
#endif

// Leaf feedback timeout (ms) - max time without 0x1DA update
#ifndef METASENSE_LEAF_VCM_FEEDBACK_TIMEOUT_MS
#define METASENSE_LEAF_VCM_FEEDBACK_TIMEOUT_MS 250
#endif

// Precharge timeout (ms) - max time to reach HV ready
#ifndef METASENSE_LEAF_VCM_PRECHARGE_TIMEOUT_MS
#define METASENSE_LEAF_VCM_PRECHARGE_TIMEOUT_MS 2500
#endif

// HV OK settle time (ms) - stabilize voltage before declaring ready
#ifndef METASENSE_LEAF_VCM_HV_OK_SETTLE_MS
#define METASENSE_LEAF_VCM_HV_OK_SETTLE_MS 100
#endif

// Fault recovery wait time (ms)
#ifndef METASENSE_LEAF_VCM_FAULT_RECOVER_MS
#define METASENSE_LEAF_VCM_FAULT_RECOVER_MS 500
#endif

// Leaf simulation feedback period (ms)
#ifndef METASENSE_LEAF_SIM_FEEDBACK_PERIOD_MS
#define METASENSE_LEAF_SIM_FEEDBACK_PERIOD_MS 20
#endif

// Leaf simulation boot delay (ms) - time before first feedback
#ifndef METASENSE_LEAF_SIM_BOOT_DELAY_MS
#define METASENSE_LEAF_SIM_BOOT_DELAY_MS 1000
#endif

// Leaf simulation ready delay (ms) - time to reach ready state
#ifndef METASENSE_LEAF_SIM_READY_DELAY_MS
#define METASENSE_LEAF_SIM_READY_DELAY_MS 1800
#endif

// ============================================================================
// MISCELLANEOUS SETTINGS
// ============================================================================

// Relative humidity offset (percent, for climate compensation)
#ifndef METASENSE_SETTINGS_RH_OFFSET_PCT
#define METASENSE_SETTINGS_RH_OFFSET_PCT 0.0f
#endif

// Force VCU ready state for UI testing (bypass safety checks)
#ifndef METASENSE_SETTINGS_FORCE_VCU_READY_FOR_UI_TEST
#define METASENSE_SETTINGS_FORCE_VCU_READY_FOR_UI_TEST false
#endif

// Enable Leaf simulation feedback by default
#ifndef METASENSE_SETTINGS_LEAF_SIM_FEEDBACK_ENABLED
#define METASENSE_SETTINGS_LEAF_SIM_FEEDBACK_ENABLED (METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS != 0)
#endif

// Use CAN-derived RPM (vs. tachometer)
#ifndef METASENSE_SETTINGS_USE_CAN_LEAF_RPM
#define METASENSE_SETTINGS_USE_CAN_LEAF_RPM true
#endif

// ============================================================================
// TELEMETRY & MONITORING PERIODS
// ============================================================================

// Ambient sensor sample period (ms)
#ifndef METASENSE_AMBIENT_SAMPLE_PERIOD_MS
#define METASENSE_AMBIENT_SAMPLE_PERIOD_MS 1000
#endif

// WebSocket publish period (fast telemetry, ms)
#ifndef METASENSE_WS_FAST_PERIOD_MS
#define METASENSE_WS_FAST_PERIOD_MS 50
#endif

// WebSocket slow publish period (ms)
#ifndef METASENSE_WS_SLOW_PERIOD_MS
#define METASENSE_WS_SLOW_PERIOD_MS 500
#endif

#endif // SETTINGSCONFIG_H
