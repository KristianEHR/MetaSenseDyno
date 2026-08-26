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
// Production: firmware actively controls RB+ relay
#define METASENSE_RBPLUS_RELAY_ENABLED 1

#endif // METASENSE_HARDWAREOUTPUT_CONFIG_H
