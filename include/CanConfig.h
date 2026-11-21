#ifndef CANCONFIG_H_
#define CANCONFIG_H_

// ===================================================================
// CAN Bus Configuration Defaults
// ===================================================================

// CAN Bus Enable/Disable
#define METASENSE_LEAF_CAN_RX_ENABLED 1
#define METASENSE_LEAF_CAN_TX_ENABLED 1

// CAN Bus Mode Configuration (required by code)
#define METASENSE_LEAF_CAN_LISTEN_ONLY 0

// Enable CAN Monitor Mode for full diagnostics
#define METASENSE_CAN_MONITOR_MODE 1

// DIAGNOSTIC: Serial (USB) debug prints for pacer, RPM decode and CAN stats.
// Set to 1 to enable. Leave 0 so the USB monitor only shows the Serial0 output.
#define METASENSE_CAN_SERIAL_DIAG 0

// DIAGNOSTIC: Log every 0x1D4 frame transmission with payload and CRC
// Set to 1 to print [0x1D4-TX] line on serial for each sent frame (verbose, 100 lines/sec)
#define METASENSE_LEAF_1D4_TX_LOG_EVERY_FRAME 0

// CAN Bus Pin Configuration
#define METASENSE_LEAF_CAN_TX_PIN 4
#define METASENSE_LEAF_CAN_RX_PIN 5
#define METASENSE_LEAF_CAN_BITRATE_KBPS 500

// CAN Bus Queue Configuration (for high-frequency traffic)
#define METASENSE_LEAF_CAN_RX_QUEUE_LEN 128
#define METASENSE_LEAF_CAN_TX_QUEUE_LEN 128

// System Timing
#define METASENSE_HEARTBEAT_PERIOD_MS 15000

// Verbose 15 s heartbeat line (WiFi, CAN, sensor snapshot). Off by default.
#ifndef METASENSE_HEARTBEAT_DETAIL
#define METASENSE_HEARTBEAT_DETAIL 0
#endif

// Frame Configuration: 0x1D4 (Torque Command)
// DBC: SG_ MotorAmpTorqueRequest : 23|12@0- (0.25,0) "Nm"
// 12-bit signed: range -2048 to 2047 → -512 to 511.75 Nm
#define METASENSE_LEAF_1D4_TORQUE_LSB_NM 0.25f
#define METASENSE_LEAF_1D4_TEMPLATE_B0 0xF7U
#define METASENSE_LEAF_1D4_TEMPLATE_B1 0x07U
#define METASENSE_LEAF_1D4_TEMPLATE_B6 0x44U

// Frame Configuration: 0x11A (Keep-Alive)
// Sent every 10ms paired with 0x1D4 (same cadence, no separate timing)
#define METASENSE_LEAF_11A_TX_ENABLED 1
#define METASENSE_LEAF_11A_TX_PERIOD_MS 10
#define METASENSE_LEAF_11A_TEMPLATE_BOOTSTRAP_ENABLE 1

// Frame Field Values
#define METASENSE_LEAF_11A_FORCE_GEAR 4
#define METASENSE_LEAF_11A_FORCE_CARONOFF 2

// 0x11A Template Bytes (with GEAR=4, CAR=2, ECO=0 encoded)
// B0: 0x4E = 01001110 (bits 4-7 = 0100 = GEAR 4)
// B1: 0x40 = 01000000 (bit 4 = ECO 0, bits 5-7 = 010 = CAR 2)
#define METASENSE_LEAF_11A_TEMPLATE_B0 0x4EU
#define METASENSE_LEAF_11A_TEMPLATE_B1 0x40U
#define METASENSE_LEAF_11A_TEMPLATE_B2 0x00U
#define METASENSE_LEAF_11A_TEMPLATE_B3 0xAAU
#define METASENSE_LEAF_11A_TEMPLATE_B4 0xC0U
#define METASENSE_LEAF_11A_TEMPLATE_B5 0x00U
#define METASENSE_LEAF_11A_TEMPLATE_M0_B7 0x6BU
#define METASENSE_LEAF_11A_TEMPLATE_M1_B7 0xEEU
#define METASENSE_LEAF_11A_TEMPLATE_M2_B7 0xE4U
#define METASENSE_LEAF_11A_TEMPLATE_M3_B7 0x61U

// Telemetry Configuration: a single WebSocket JSON message (one common
// tier, all fields, no per-field change-detection) is sent to every
// connected client at this cadence. Sends are per-client: a client whose
// library queue is backed up is skipped for that tick, so its connection
// stays open and catches up. Without this, the library closes the TCP
// connection without a close frame when WS_MAX_QUEUED_MESSAGES is reached.
// See notifyClients() in Input.cpp.
#define METASENSE_WS_FAST_PERIOD_MS 33

// Live gauge/chart fields go in a fast frame at METASENSE_WS_FAST_PERIOD_MS.
// Digital displays and CAN/status fields go in a separate slow frame at this
// period, which keeps the fast frame small enough to avoid client congestion.
#define METASENSE_WS_SLOW_PERIOD_MS 100

// Simulation Feedback (required by code)
#define METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS 0

// Derived CAN configuration (validation)
#if METASENSE_LEAF_CAN_RX_ENABLED == 0 && METASENSE_LEAF_CAN_TX_ENABLED == 0
#error "CAN bus must have RX or TX enabled"
#endif

#endif // CANCONFIG_H_
