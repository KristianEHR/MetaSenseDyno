#ifndef METASENSE_VCU_CONFIG_H
#define METASENSE_VCU_CONFIG_H

// VCU Ready Source Configuration
// ================================
// Determines how the dyno controller detects VCU readiness

// VCU ready source switch:
// 1 = use GPIO RB+ input (normal operation)
// 0 = force VCU ready true (bench testing without VCU)
#if !defined(VCU_switch) && defined(VCU_set)
#define VCU_switch VCU_set
#endif

#ifndef VCU_switch
#define VCU_switch 0
#endif

// HV R+ Precharge Control
// 1 = VCU owns HV R+ precharge relay control
// 0 = DYNO owns HV R+ precharge relay control (default)
#ifndef METASENSE_VCU_OWNS_HV_RPLUS_PRECHARGE
#define METASENSE_VCU_OWNS_HV_RPLUS_PRECHARGE 0
#endif

// VCU Simulation Mode
// 1 = simulate VCU inputs (for bench testing without VCU)
// 0 = normal operation (use real GPIO/CAN inputs)
#ifndef METASENSE_VCU_SIM_MODE
#define METASENSE_VCU_SIM_MODE 0
#endif

#endif // METASENSE_VCU_CONFIG_H
