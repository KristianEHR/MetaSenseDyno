#ifndef CANCONFIG_H_
#define CANCONFIG_H_

// CAN Bus Configuration (consolidated from main.cpp and platformio.ini)
// ===================================================================

#ifndef METASENSE_LEAF_CAN_RX_ENABLED
#define METASENSE_LEAF_CAN_RX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_CAN_TX_ENABLED
#define METASENSE_LEAF_CAN_TX_ENABLED 0
#endif

#ifndef METASENSE_LEAF_CAN_LISTEN_ONLY
#define METASENSE_LEAF_CAN_LISTEN_ONLY 0
#endif

#ifndef METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS
#define METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS 0
#endif

#ifndef METASENSE_LEAF_CAN_TX_PIN
#define METASENSE_LEAF_CAN_TX_PIN 4
#endif

#ifndef METASENSE_LEAF_CAN_RX_PIN
#define METASENSE_LEAF_CAN_RX_PIN 5
#endif

#ifndef METASENSE_LEAF_CAN_BITRATE_KBPS
#define METASENSE_LEAF_CAN_BITRATE_KBPS 500
#endif

// Derived CAN configuration (validation)
#if METASENSE_LEAF_CAN_RX_ENABLED == 0 && METASENSE_LEAF_CAN_TX_ENABLED == 0
#error "CAN bus must have RX or TX enabled"
#endif

#endif // CANCONFIG_H_
