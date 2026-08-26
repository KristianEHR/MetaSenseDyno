// Hardware Output & Relay Control Configuration
// ===============================================
// Centralized configuration for relay control, SSR switching,
// and precharge sequencing

#ifndef METASENSE_HARDWAREOUTPUT_CONFIG_H
#define METASENSE_HARDWAREOUTPUT_CONFIG_H

// ============================================================================
// Relay Control & Enable Flags
// ============================================================================
// RB+ relay control (R+ means relay plus / boost stage)
// 0 = firmware releases GPIO from control (default)
// 1 = firmware actively controls RB+ relay
#ifndef METASENSE_RBPLUS_RELAY_ENABLED
#define METASENSE_RBPLUS_RELAY_ENABLED 0
#endif

#endif // METASENSE_HARDWAREOUTPUT_CONFIG_H
