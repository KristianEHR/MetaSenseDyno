#ifndef METASENSE_BUILD_CONFIG_H
#define METASENSE_BUILD_CONFIG_H

#include <cstdint>

// Runtime Instrumentation
// ================================
// Enable/disable runtime task timing measurements
#define ENABLE_RUNTIME_INSTRUMENTATION 0

// Firmware Identification
// ================================
#ifndef METASENSE_FW_ID
#define METASENSE_FW_ID "unknown"
#endif

// Task Scheduling Periods
// ================================
// Control loop update period (milliseconds)
#ifndef METASENSE_CONTROL_PERIOD_MS
#define METASENSE_CONTROL_PERIOD_MS 10
#endif

// Heartbeat/status report period (milliseconds)
#ifndef METASENSE_HEARTBEAT_PERIOD_MS
#define METASENSE_HEARTBEAT_PERIOD_MS 2000
#endif

// Modbus poll period (fixed, not configurable)
constexpr uint32_t METASENSE_MODBUS_PERIOD_MS = 50;

// Startup Sniff Configuration
// ================================
#ifndef METASENSE_STARTUP_SNIFF_RELEASE_TX_PIN
#define METASENSE_STARTUP_SNIFF_RELEASE_TX_PIN -1
#endif

// Derived Timing Constants (for use in code)
// ================================
constexpr uint32_t kControlPeriodMs = METASENSE_CONTROL_PERIOD_MS;
constexpr uint32_t kModbusPeriodMs = METASENSE_MODBUS_PERIOD_MS;
constexpr uint32_t kHeartbeatPeriodMs = METASENSE_HEARTBEAT_PERIOD_MS;

#endif // METASENSE_BUILD_CONFIG_H
