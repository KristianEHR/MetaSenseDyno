#ifndef METASENSE_FEATURE_FLAGS_H
#define METASENSE_FEATURE_FLAGS_H

// Feature Flags and Diagnostic Control
// ================================
// Production and development feature toggles

// Leaf 0x120 Frame Handling
// ================================

// Enable VCM checklist mode (extended diagnostics)
#ifndef METASENSE_LEAF_VCM_CHECKLIST_MODE
#define METASENSE_LEAF_VCM_CHECKLIST_MODE 1
#endif

// VCM diagnostic output
#ifndef METASENSE_LEAF_VCM_DIAGNOSTICS
#define METASENSE_LEAF_VCM_DIAGNOSTICS 0
#endif

// Legacy verbose VCM RX logging
#ifndef METASENSE_LEAF_VCM_LEGACY_VERBOSE_LOGS
#define METASENSE_LEAF_VCM_LEGACY_VERBOSE_LOGS 0
#endif

// VCM RX warning logs (CRC, missing frames)
#ifndef METASENSE_LEAF_VCM_RX_WARN_LOGS
#define METASENSE_LEAF_VCM_RX_WARN_LOGS 0
#endif

// CRC Algorithm Deep Diagnostics
// Deep CRC algorithm tracing and validation (very verbose)
#ifndef METASENSE_LEAF_CRC_DEEP_LOGS
#define METASENSE_LEAF_CRC_DEEP_LOGS 0
#endif

// Telemetry and Monitoring
// ================================

// Simple telemetry monitoring (core metrics only)
#ifndef METASENSE_LEAF_MONITOR_SIMPLE_LOGS
#define METASENSE_LEAF_MONITOR_SIMPLE_LOGS 1
#endif

// Decimation factor for simple monitoring output
#ifndef METASENSE_LEAF_MONITOR_SIMPLE_DECIMATE
#define METASENSE_LEAF_MONITOR_SIMPLE_DECIMATE 20U
#endif

// CAN Bus Feedback Simulation
// ================================
// Simulate Leaf CAN feedback without physical bus
// (useful for lab testing without full CAN network)
#ifndef METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS
#define METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS 0
#endif

// CAN RPM Monitoring
// ================================
// Minimum time between CAN RPM updates (milliseconds)
#ifndef METASENSE_CAN_RPM_MIN_UPDATE_MS
#define METASENSE_CAN_RPM_MIN_UPDATE_MS 10
#endif

// JSON Telemetry
// ================================
// Enable/disable JSON-format telemetry output
#ifndef METASENSE_CAN_MONITOR_JSON_ENABLED
#define METASENSE_CAN_MONITOR_JSON_ENABLED 0
#endif

#endif // METASENSE_FEATURE_FLAGS_H
