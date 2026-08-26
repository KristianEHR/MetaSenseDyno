// CAN Bus Diagnostics & Logging Configuration
// ============================================
// Centralized configuration for CAN message logging, frame capture,
// and startup sniffing diagnostics

#ifndef METASENSE_CANDIAG_CONFIG_H
#define METASENSE_CANDIAG_CONFIG_H

// ============================================================================
// CAN Message Logging Levels
// ============================================================================
#ifndef METASENSE_CAN_LOG_KEY_FRAMES
#define METASENSE_CAN_LOG_KEY_FRAMES 0
#endif

#ifndef METASENSE_CAN_LOG_ALL_FRAMES
#define METASENSE_CAN_LOG_ALL_FRAMES 0
#endif

#ifndef METASENSE_CAN_LOG_11A_CHANGES
#define METASENSE_CAN_LOG_11A_CHANGES 0
#endif

// ============================================================================
// CAN Diagnostics & Monitoring Options
// ============================================================================
#ifndef METASENSE_CAN_ID_SCAN
#define METASENSE_CAN_ID_SCAN 1
#endif

#ifndef METASENSE_CAN_SNIFF_ONLY
#define METASENSE_CAN_SNIFF_ONLY 0
#endif

#ifndef METASENSE_CAN_FRAME_IDENTIFIER
#define METASENSE_CAN_FRAME_IDENTIFIER 0
#endif

#ifndef METASENSE_CAN_RX_ONE_LINE_LOG
#define METASENSE_CAN_RX_ONE_LINE_LOG 0
#endif

// ============================================================================
// Startup Sniff Capture Configuration
// (Used for boot-time CAN frame analysis and diagnostics)
// ============================================================================
#ifndef METASENSE_STARTUP_RX_ONLY_MODE
#define METASENSE_STARTUP_RX_ONLY_MODE 0
#endif

#ifndef METASENSE_STARTUP_SNIFF_CAPTURE_MS
#define METASENSE_STARTUP_SNIFF_CAPTURE_MS 2000U
#endif

#ifndef METASENSE_STARTUP_SNIFF_CAPTURE_MAX_FRAMES
#define METASENSE_STARTUP_SNIFF_CAPTURE_MAX_FRAMES 2048U
#endif

#ifndef METASENSE_STARTUP_SNIFF_ARM_DELAY_MS
#define METASENSE_STARTUP_SNIFF_ARM_DELAY_MS 0U
#endif

#ifndef METASENSE_STARTUP_SNIFF_START_ON_FIRST_11A
#define METASENSE_STARTUP_SNIFF_START_ON_FIRST_11A 0U
#endif

#ifndef METASENSE_STARTUP_SNIFF_REARM_ON_11A_GAP_MS
#define METASENSE_STARTUP_SNIFF_REARM_ON_11A_GAP_MS 0U
#endif

#ifndef METASENSE_STARTUP_SNIFF_DUMP_DELAY_MS
#define METASENSE_STARTUP_SNIFF_DUMP_DELAY_MS 30000U
#endif

#ifndef METASENSE_STARTUP_SNIFF_SAVE_PATH
#define METASENSE_STARTUP_SNIFF_SAVE_PATH "/captures/startup_11a.csv"
#endif

// ============================================================================
// 0x1D4 Torque Command Sniffing & Buffering
// ============================================================================
#ifndef METASENSE_LEAF_1D4_SNIFF_RX_ENABLED
#define METASENSE_LEAF_1D4_SNIFF_RX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_1D4_RINGBUF_ENABLE
#define METASENSE_LEAF_1D4_RINGBUF_ENABLE 0
#endif

#ifndef METASENSE_LEAF_1D4_RINGBUF_SIZE
#define METASENSE_LEAF_1D4_RINGBUF_SIZE 64U
#endif

#endif // METASENSE_CANDIAG_CONFIG_H
