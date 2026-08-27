#ifndef CANCONFIG_H_
#define CANCONFIG_H_

// ===================================================================
// CAN Bus Configuration Defaults (Non-Zero Values Only)
// Consolidated from platformio.ini -D flags where value > 0
// All can be overridden via -D compiler flags if needed
// ===================================================================

// CAN Bus Enable/Disable - ACTIVE (=1)
#ifndef METASENSE_LEAF_CAN_RX_ENABLED
#define METASENSE_LEAF_CAN_RX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_CAN_TX_ENABLED
#define METASENSE_LEAF_CAN_TX_ENABLED 1
#endif

// CAN Bus Mode Configuration (required by code)
#ifndef METASENSE_LEAF_CAN_LISTEN_ONLY
#define METASENSE_LEAF_CAN_LISTEN_ONLY 0
#endif

// CAN Bus Pin Configuration
#ifndef METASENSE_LEAF_CAN_TX_PIN
#define METASENSE_LEAF_CAN_TX_PIN 4
#endif

#ifndef METASENSE_LEAF_CAN_RX_PIN
#define METASENSE_LEAF_CAN_RX_PIN 5
#endif

#ifndef METASENSE_LEAF_CAN_BITRATE_KBPS
#define METASENSE_LEAF_CAN_BITRATE_KBPS 500
#endif

// CAN Bus Queue Configuration (for high-frequency traffic)
#ifndef METASENSE_LEAF_CAN_RX_QUEUE_LEN
#define METASENSE_LEAF_CAN_RX_QUEUE_LEN 128
#endif

#ifndef METASENSE_LEAF_CAN_TX_QUEUE_LEN
#define METASENSE_LEAF_CAN_TX_QUEUE_LEN 128
#endif

// Leaf Variant Configuration - ACTIVE (=1)
#ifndef METASENSE_55A
#define METASENSE_55A 1
#endif

// System Timing - ACTIVE (>0)
#ifndef METASENSE_HEARTBEAT_PERIOD_MS
#define METASENSE_HEARTBEAT_PERIOD_MS 15000
#endif

// Hardware Configuration - ACTIVE (=1)
#ifndef METASENSE_RBPLUS_RELAY_ENABLED
#define METASENSE_RBPLUS_RELAY_ENABLED 1
#endif

// Frame Configuration: 0x1D4 (Torque Command) - ACTIVE (>0)
// DBC: SG_ MotorAmpTorqueRequest : 23|12@0- (0.25,0) "Nm"
// 12-bit signed: range -2048 to 2047 → -512 to 511.75 Nm
#ifndef METASENSE_LEAF_1D4_TORQUE_LSB_NM
#define METASENSE_LEAF_1D4_TORQUE_LSB_NM 0.25f
#endif

#ifndef METASENSE_LEAF_1D4_TEMPLATE_B0
#define METASENSE_LEAF_1D4_TEMPLATE_B0 0xF7U
#endif

#ifndef METASENSE_LEAF_1D4_TEMPLATE_B1
#define METASENSE_LEAF_1D4_TEMPLATE_B1 0x07U
#endif

#ifndef METASENSE_LEAF_1D4_TEMPLATE_B6
#define METASENSE_LEAF_1D4_TEMPLATE_B6 0x44U
#endif

// Frame Configuration: 0x11A (Keep-Alive) - ACTIVE (=1)
// Sent every 10ms paired with 0x1D4 (same cadence, no separate timing)
#ifndef METASENSE_LEAF_11A_TX_ENABLED
#define METASENSE_LEAF_11A_TX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_11A_TX_PERIOD_MS
#define METASENSE_LEAF_11A_TX_PERIOD_MS 10
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_BOOTSTRAP_ENABLE
#define METASENSE_LEAF_11A_TEMPLATE_BOOTSTRAP_ENABLE 1
#endif

// Frame Field Values - ACTIVE (>0)
#ifndef METASENSE_LEAF_11A_FORCE_GEAR
#define METASENSE_LEAF_11A_FORCE_GEAR 4
#endif

#ifndef METASENSE_LEAF_11A_FORCE_CARONOFF
#define METASENSE_LEAF_11A_FORCE_CARONOFF 2
#endif

#ifndef METASENSE_LEAF_11A_FORCE_ECO
#define METASENSE_LEAF_11A_FORCE_ECO 0
#endif

// 0x11A Template Bytes (with GEAR=4, CAR=2, ECO=0 encoded)
// B0: 0x4E = 01001110 (bits 4-7 = 0100 = GEAR 4)
// B1: 0x40 = 01000000 (bit 4 = ECO 0, bits 5-7 = 010 = CAR 2)
#ifndef METASENSE_LEAF_11A_TEMPLATE_B0
#define METASENSE_LEAF_11A_TEMPLATE_B0 0x4EU
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_B1
#define METASENSE_LEAF_11A_TEMPLATE_B1 0x40U
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_B2
#define METASENSE_LEAF_11A_TEMPLATE_B2 0x00U
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_B3
#define METASENSE_LEAF_11A_TEMPLATE_B3 0xAAU
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_B4
#define METASENSE_LEAF_11A_TEMPLATE_B4 0xC0U
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_B5
#define METASENSE_LEAF_11A_TEMPLATE_B5 0x00U
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_M0_B7
#define METASENSE_LEAF_11A_TEMPLATE_M0_B7 0x6BU
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_M1_B7
#define METASENSE_LEAF_11A_TEMPLATE_M1_B7 0xEEU
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_M2_B7
#define METASENSE_LEAF_11A_TEMPLATE_M2_B7 0xE4U
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_M3_B7
#define METASENSE_LEAF_11A_TEMPLATE_M3_B7 0x61U
#endif

// Telemetry Configuration
// METASENSE_CAN_MONITOR_MODE: 0=production (16ms, NO metrics), 1=debug (50ms WITH detailed CAN metrics)
// Mode 0: Sends only core telemetry (rpm, power, temps) - minimal CPU/WiFi load
// Mode 1: Sends full metrics including leaf_1da/1d4/11a CAN frame data - for debugging/monitoring
#ifndef METASENSE_CAN_MONITOR_MODE
#define METASENSE_CAN_MONITOR_MODE 0
#endif

// Only enable CAN metrics JSON when monitor mode = 1
#ifndef METASENSE_CAN_MONITOR_JSON_ENABLED
#define METASENSE_CAN_MONITOR_JSON_ENABLED (METASENSE_CAN_MONITOR_MODE == 1)
#endif

// Telemetry period (ms): 16ms for production, 50ms for debug/monitor mode
#ifndef METASENSE_WS_FAST_PERIOD_MS
#if METASENSE_CAN_MONITOR_MODE == 1
#define METASENSE_WS_FAST_PERIOD_MS 50
#else
#define METASENSE_WS_FAST_PERIOD_MS 16
#endif
#endif

// Simulation Feedback (required by code)
#ifndef METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS
#define METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS 0
#endif

// Derived CAN configuration (validation)
#if METASENSE_LEAF_CAN_RX_ENABLED == 0 && METASENSE_LEAF_CAN_TX_ENABLED == 0
#error "CAN bus must have RX or TX enabled"
#endif

#endif // CANCONFIG_H_
