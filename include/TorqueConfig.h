#ifndef METASENSE_TORQUE_CONFIG_H
#define METASENSE_TORQUE_CONFIG_H

// Torque Control Configuration
// ================================

// Torque Override (Test Mode)
// Enable manual torque override for testing
#ifndef METASENSE_TEST_TORQUE_OVERRIDE_ENABLED
#define METASENSE_TEST_TORQUE_OVERRIDE_ENABLED 0
#endif

// Fixed torque value for override mode (Nm)
#ifndef METASENSE_TEST_TORQUE_OVERRIDE_NM
#define METASENSE_TEST_TORQUE_OVERRIDE_NM 25.0f
#endif

// Torque Step Sequencer (automated ramp testing)
// Enable automated torque stepping sequences
#ifndef METASENSE_TORQUE_STEP_SEQUENCER_ENABLED
#define METASENSE_TORQUE_STEP_SEQUENCER_ENABLED 0
#endif

// Step size for sequencer (Nm per step)
#ifndef METASENSE_TORQUE_STEP_SEQUENCER_STEP_NM
#define METASENSE_TORQUE_STEP_SEQUENCER_STEP_NM 0.5f
#endif

// Maximum torque for sequencer (Nm)
#ifndef METASENSE_TORQUE_STEP_SEQUENCER_MAX_NM
#define METASENSE_TORQUE_STEP_SEQUENCER_MAX_NM 3.0f
#endif

// Dwell time at each step (milliseconds)
#ifndef METASENSE_TORQUE_STEP_SEQUENCER_DWELL_MS
#define METASENSE_TORQUE_STEP_SEQUENCER_DWELL_MS 5000U
#endif

// Leaf 0x120 Torque Command Calibration
// ================================
// Base calibration: torque command (Nm) = (raw * SLOPE) + OFFSET

// Linear slope for 0x120 torque calibration
#ifndef METASENSE_LEAF_120_CMD_BASE_SLOPE
#define METASENSE_LEAF_120_CMD_BASE_SLOPE 0.0300f
#endif

// Offset for 0x120 torque calibration (Nm)
#ifndef METASENSE_LEAF_120_CMD_BASE_OFFSET_NM
#define METASENSE_LEAF_120_CMD_BASE_OFFSET_NM 0.000f
#endif

// Minimum number of samples before curve fitting is valid
#ifndef METASENSE_LEAF_120_CMD_MIN_FIT_SAMPLES
#define METASENSE_LEAF_120_CMD_MIN_FIT_SAMPLES 128U
#endif

#endif // METASENSE_TORQUE_CONFIG_H
