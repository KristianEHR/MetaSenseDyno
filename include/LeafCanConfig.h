#ifndef METASENSE_LEAFCANCONFIG_H
#define METASENSE_LEAFCANCONFIG_H

/**
 * @file LeafCanConfig.h
 * @brief Nissan Leaf CAN frame encoding/decoding parameters
 * 
 * Centralizes all Leaf-specific CAN protocol constants including:
 * - Frame timing (0x1D4, 0x11A transmission rates)
 * - Encoding parameters (torque scales, bit positions, Motorola/Intel layouts)
 * - CRC algorithms (polynomial 0x85 for both 0x1DA and 0x1D4)
 * - Keep-alive/echo protocol specifics
 * - Mode selection and control flags
 * 
 * Extracted from: src/Input.cpp, src/LeafCan.cpp
 */

// ===== CAN MODE CONFIGURATION =====
// Frame reception/transmission control flags

#ifndef METASENSE_LEAF_TORQUE_TRACK_TIMEOUT_MS
#define METASENSE_LEAF_TORQUE_TRACK_TIMEOUT_MS 3000
#endif

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

// ===== FRAME TIMING =====
// 0x1D4 Torque Command (TX to Inverter)
#ifndef METASENSE_LEAF_1D4_TX_PERIOD_MS
#define METASENSE_LEAF_1D4_TX_PERIOD_MS 10U  // 100 Hz transmission rate (every 10ms)
#endif

// 0x1D4 Torque payload state machine update period
#ifndef METASENSE_LEAF_1D4_TORQUE_PAYLOAD_UPDATE_PERIOD_MS
#define METASENSE_LEAF_1D4_TORQUE_PAYLOAD_UPDATE_PERIOD_MS 100U  // Update torque payload every 100ms
#endif

// 0x11A Keep-Alive/Echo (TX to Inverter)
#ifndef METASENSE_LEAF_11A_TX_PERIOD_MS
#define METASENSE_LEAF_11A_TX_PERIOD_MS 20U  // Keep-alive transmission period (ms)
#endif

#ifndef METASENSE_LEAF_11A_TX_ENABLED
#define METASENSE_LEAF_11A_TX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_11A_FORCE_GEAR
#define METASENSE_LEAF_11A_FORCE_GEAR 4U  // Default fixed gear for 0x11A transmit
#endif

#ifndef METASENSE_LEAF_11A_FORCE_CARONOFF
#define METASENSE_LEAF_11A_FORCE_CARONOFF 2U  // Default fixed CarOnOff for 0x11A transmit
#endif

#ifndef METASENSE_LEAF_11A_TEMPLATE_BOOTSTRAP_ENABLE
#define METASENSE_LEAF_11A_TEMPLATE_BOOTSTRAP_ENABLE 0
#endif

// 0x11A Transmit Template Bytes
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

// ===== 0x1D4 TORQUE COMMAND ENCODING & TRANSMISSION =====
// SG_ MotorAmpTorqueRequest : 23|12@0- (0.0625,0) "Nm"
// Motorola byte order, 12-bit signed, scale 0.0625 Nm/LSB

#ifndef METASENSE_LEAF_1D4_TORQUE_LSB_NM
#define METASENSE_LEAF_1D4_TORQUE_LSB_NM 0.0625f  // 1 LSB = 0.0625 Nm
#endif

#ifndef METASENSE_LEAF_1D4_TORQUE_START_BIT
#define METASENSE_LEAF_1D4_TORQUE_START_BIT 23U  // Motorola bit numbering
#endif

#ifndef METASENSE_LEAF_1D4_TORQUE_LENGTH
#define METASENSE_LEAF_1D4_TORQUE_LENGTH 12U  // 12-bit two's complement signed
#endif

// 0x1D4 CRC Clock Configuration
#ifndef METASENSE_LEAF_1D4_CRC_CLOCK_XOR_ENABLE
#define METASENSE_LEAF_1D4_CRC_CLOCK_XOR_ENABLE 1  // XOR CRC by HCM clock bin
#endif

// 0x1D4 RX Sniffing & Analysis
#ifndef METASENSE_LEAF_1D4_SNIFF_RX_ENABLED
#define METASENSE_LEAF_1D4_SNIFF_RX_ENABLED 1  // Capture incoming 0x1D4 for analysis
#endif

#ifndef METASENSE_LEAF_1D4_RAW_SNIFF_ONLY
#define METASENSE_LEAF_1D4_RAW_SNIFF_ONLY 0
#endif

// 0x1D4 TX Mode Selection
#ifndef METASENSE_LEAF_1D4_TEMPLATE_TX_MODE
// 0: Build 0x1D4 from logic
// 1: Thunderstruck template (patch torque + clock + CRC only)
// 2: Replay captured clock-continuous loop
// 3: Ring-buffer newest frame as TX base, patch fields
// 4: Strict raw replay from ring-buffer (no patching)
#define METASENSE_LEAF_1D4_TEMPLATE_TX_MODE 0
#endif

#ifndef METASENSE_LEAF_1D4_RING_TX_SOURCE_AGE
#define METASENSE_LEAF_1D4_RING_TX_SOURCE_AGE 0  // 0=newest ring frame
#endif

#ifndef METASENSE_LEAF_1D4_REPLAY_RECALC_CRC
#define METASENSE_LEAF_1D4_REPLAY_RECALC_CRC 1  // Recalc CRC on replay
#endif

#ifndef METASENSE_LEAF_1D4_TEMPLATE_MAX_ABS_TORQUE_NM
#define METASENSE_LEAF_1D4_TEMPLATE_MAX_ABS_TORQUE_NM 3.75f  // Conservative region for template TX
#endif

// 0x1D4 Template/Header Bytes (from Thunderstruck reference)
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

// 0x1D4 Keep-Alive Zero-Crossing Pulse Configuration
#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_REFRESH_MS
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_REFRESH_MS 500U  // Periodically refresh inverter state
#endif

#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_PULSE_MS
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_PULSE_MS 20U  // Keep negative pulse long enough
#endif

#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_POS_NM
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_POS_NM 0.60f  // Positive torque value
#endif

#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_PRE_POS_MS
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_PRE_POS_MS 20U  // Pre-pulse positive duration
#endif

#ifndef METASENSE_LEAF_1D4_KEEPALIVE_ZERO_POST_POS_MS
#define METASENSE_LEAF_1D4_KEEPALIVE_ZERO_POST_POS_MS 20U  // Post-pulse positive duration
#endif

// ===== 0x1DA MOTOR FEEDBACK DECODING (RX from Inverter) =====
// Frame ID 0x1DA contains motor speed, torque, and status from Nissan inverter

// Actual Motor Torque: SG_ MG_EffectiveTorque : 18|11@0- (0.5,0) "Nm"
// Motorola byte order, 11-bit signed, scale 0.5 Nm/LSB
#ifndef METASENSE_LEAF_1DA_TORQUE_LSB_NM
#define METASENSE_LEAF_1DA_TORQUE_LSB_NM 0.5f  // 1 LSB = 0.5 Nm
#endif

#ifndef METASENSE_LEAF_1DA_TORQUE_START_BIT
#define METASENSE_LEAF_1DA_TORQUE_START_BIT 18U  // Motorola bit numbering
#endif

#ifndef METASENSE_LEAF_1DA_TORQUE_LENGTH
#define METASENSE_LEAF_1DA_TORQUE_LENGTH 11U  // 11-bit two's complement signed
#endif

// Motor Speed: SG_ MG_OutputRevolution : 39|15@0- (1,0) "rpm"
// Motorola byte order, 15-bit signed, scale 1 rpm/LSB
#ifndef METASENSE_LEAF_1DA_RPM_START_BIT
#define METASENSE_LEAF_1DA_RPM_START_BIT 39U  // Motorola bit numbering
#endif

#ifndef METASENSE_LEAF_1DA_RPM_LENGTH
#define METASENSE_LEAF_1DA_RPM_LENGTH 15U  // 15-bit two's complement signed
#endif

#ifndef METASENSE_LEAF_1DA_RPM_SCALE
#define METASENSE_LEAF_1DA_RPM_SCALE 1.0f  // 1 LSB = 1 rpm
#endif

// Input Voltage (DC Link): SG_ MG_InputVoltage : 0|8@1+ (2,0) "V"
// Intel byte order (little-endian), 8-bit unsigned, scale 2.0V/LSB
#ifndef METASENSE_LEAF_1DA_VOLTAGE_START_BIT
#define METASENSE_LEAF_1DA_VOLTAGE_START_BIT 0U  // Byte 0 of frame
#endif

#ifndef METASENSE_LEAF_1DA_VOLTAGE_SCALE
#define METASENSE_LEAF_1DA_VOLTAGE_SCALE 2.0f  // 1 LSB = 2.0V
#endif

// Motor Speed Redundant Field (fallback): Uses bytes [2:3] when primary is zero
#ifndef METASENSE_LEAF_1DA_SPEED_ALT_BYTE_LO
#define METASENSE_LEAF_1DA_SPEED_ALT_BYTE_LO 2U  // Byte 2 (low)
#endif

#ifndef METASENSE_LEAF_1DA_SPEED_ALT_BYTE_HI
#define METASENSE_LEAF_1DA_SPEED_ALT_BYTE_HI 3U  // Byte 3 (high)
#endif

// 0x1DA Status/Clock Fields
#ifndef METASENSE_LEAF_1DA_CLOCK_START_BIT
#define METASENSE_LEAF_1DA_CLOCK_START_BIT 48U  // Intel bits 48-49: 2-bit rolling counter
#endif

#ifndef METASENSE_LEAF_1DA_CLOCK_LENGTH
#define METASENSE_LEAF_1DA_CLOCK_LENGTH 2U  // Modulo-4 counter
#endif

#ifndef METASENSE_LEAF_1DA_ERROR_CODES_START_BIT
#define METASENSE_LEAF_1DA_ERROR_CODES_START_BIT 50U  // Intel bits 50-55: 6-bit error code
#endif

#ifndef METASENSE_LEAF_1DA_ERROR_CODES_LENGTH
#define METASENSE_LEAF_1DA_ERROR_CODES_LENGTH 6U  // 6-bit field
#endif

// ===== 0x11A KEEP-ALIVE / ECHO FRAME (TX) =====
// Minimal keep-alive frame echoed back to inverter

#ifndef METASENSE_LEAF_11A_BYTE1
#define METASENSE_LEAF_11A_BYTE1 0x40U  // Byte 1: Fixed 0x40
#endif

#ifndef METASENSE_LEAF_11A_MUX_BYTE
#define METASENSE_LEAF_11A_MUX_BYTE 6U  // Byte 6: Mux selector [0,1,2,3]
#endif

#ifndef METASENSE_LEAF_11A_MUX_VALUE_BYTE
#define METASENSE_LEAF_11A_MUX_VALUE_BYTE 7U  // Byte 7: Mux-dependent value (echoed or computed)
#endif

// ===== CAN BUS HARDWARE CONFIGURATION =====

#ifndef METASENSE_LEAF_CAN_TX_PIN
#define METASENSE_LEAF_CAN_TX_PIN 4
#endif

#ifndef METASENSE_LEAF_CAN_RX_PIN
#define METASENSE_LEAF_CAN_RX_PIN 5
#endif

#ifndef METASENSE_LEAF_CAN_MAX_FRAMES_PER_LOOP
#define METASENSE_LEAF_CAN_MAX_FRAMES_PER_LOOP 8
#endif

// ===== CAN DIAGNOSTICS & TEST FLAGS =====

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
#define METASENSE_FORCE_TACHO_RPM_SOURCE 0  // Default: prefer CAN RPM; set 1 for tachogen-only
#endif

// ===== CRC ALGORITHM PARAMETERS =====
// Both 0x1DA (RX) and 0x1D4 (TX) use polynomial 0x85 but with different configurations

// 0x1DA CRC (RX Validation from Nissan Inverter)
// Polynomial: 0x85, Initial: 0x00, XOR Output: 0xBF, Frame ID prepended
#ifndef METASENSE_LEAF_1DA_CRC_POLYNOMIAL
#define METASENSE_LEAF_1DA_CRC_POLYNOMIAL 0x85U
#endif

#ifndef METASENSE_LEAF_1DA_CRC_INIT
#define METASENSE_LEAF_1DA_CRC_INIT 0x00U
#endif

#ifndef METASENSE_LEAF_1DA_CRC_XOR_OUT
#define METASENSE_LEAF_1DA_CRC_XOR_OUT 0xBFU
#endif

#ifndef METASENSE_LEAF_1DA_CRC_INCLUDE_FRAME_ID
#define METASENSE_LEAF_1DA_CRC_INCLUDE_FRAME_ID 1  // Prepend frame ID 0xDA to payload
#endif

// 0x1D4 CRC (TX Generation to Nissan Inverter)
// Polynomial: 0x85, Initial: 0x00, XOR Output: NONE (plain), NO Frame ID prepend
// CRITICAL: Do NOT prepend frame ID and do NOT XOR output!
#ifndef METASENSE_LEAF_1D4_CRC_POLYNOMIAL
#define METASENSE_LEAF_1D4_CRC_POLYNOMIAL 0x85U
#endif

#ifndef METASENSE_LEAF_1D4_CRC_INIT
#define METASENSE_LEAF_1D4_CRC_INIT 0x00U
#endif

#ifndef METASENSE_LEAF_1D4_CRC_XOR_OUT
#define METASENSE_LEAF_1D4_CRC_XOR_OUT 0x00U  // NO XOR (critical difference from 0x1DA)
#endif

#ifndef METASENSE_LEAF_1D4_CRC_INCLUDE_FRAME_ID
#define METASENSE_LEAF_1D4_CRC_INCLUDE_FRAME_ID 0  // NO frame ID prepend (critical difference)
#endif

#endif // METASENSE_LEAFCANCONFIG_H
