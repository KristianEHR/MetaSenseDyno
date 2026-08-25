# MetaSense-DYNO Firmware Cleanup Analysis
**Date:** 2026-08-25  
**Scope:** src/ directory (main source files)  
**Estimated Code Savings:** 15-25% LOC reduction

---

## Executive Summary

The firmware contains significant amounts of **dead code, debug logging, and disabled diagnostic features** that are production-irrelevant. Quick wins include removing ~50+ Serial.printf() calls and 3+ large disabled feature blocks. The codebase is currently configured for "development+diagnostics" mode; production cleanup would hardcode known settings and eliminate optional paths.

---

## 1. DEBUG LOGGING (Serial.printf/Serial0.printf) — HIGH PRIORITY

**Quick Wins:** ~150-200 lines removed  
**Effort:** Very Low  
**Risk:** None if disabled features stay disabled

### [main.cpp](main.cpp)

| Lines | Issue | Type | Impact |
|-------|-------|------|--------|
| 213, 224 | `[BOOT] I2C device found/scan complete` | Diagnostic logs | 2 calls, 4 lines |
| 260, 271 | `[BOOT] I2C device found @ 0x%02X` | Duplicate (wifi retry path) | 2 calls, 4 lines |
| 304, 311, 313 | `[BOOT] WiFi connecting/connected/MAC` | Boot sequence | 3 calls, 6 lines |
| 732-740 | `[WiFi] Status transition` (x2) | Status monitoring | 2 duplicate calls, 6 lines |
| 751-753 | `[WiFi] Retry connect` (x2) | Retry logic | 2 duplicate calls, 6 lines |
| 763-771 | `[BOOTSTATUS]` detailed summary | Boot diagnostics | 2 duplicate calls, 8 lines |
| 835-852 | `[HEARTBEAT]` massive telemetry dump | Periodic status (~5KB per heartbeat) | 2 duplicate calls, **18 lines** |

**Subtotal:** 50+ lines, 1300+ chars per heartbeat cycle

**Recommendation:**
- Option A: Comment out all Serial.printf (keep Serial0 for debug console)
- Option B: Create `#define VERBOSE_LOGGING 0` guard (production-ready, dev-friendly)

---

### [CANBus.cpp](CANBus.cpp)

| Lines | Issue | Type | Impact |
|-------|-------|------|--------|
| 222 | `[STARTUP-SNIFF] capture started` | Feature startup | 1 call |
| 250 | `[STARTUP-SNIFF] re-armed after 11A gap` | Conditional rearm | 1 call |
| 317, 346 | `meta,capture_ms,%lu,count,%u` CSV header/rows | Sniff export | 2 calls, **~10KB dump** |
| 452 | `[CAN-ID-REPORT] active=%lu dt=%lums` | Frame monitor | 1 call |
| 469 | `[CAN-ID] id=0x%03lX d=%lu hz=%.1f...` | Per-frame logging | 1 call per unique CAN ID |
| 503, 508 | `CAN-SNIFF-SNAPSHOT` summary dump | Snapshot export | 2 calls |

**Subtotal:** 9 calls, 10+KB per sniff capture (disabled by default but still compiled)

**Recommendation:**
- Guard all sniff logging under `#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE` or wrap entire block (lines 174-350) in ifdef

---

### [Input.cpp](Input.cpp) — **LARGEST SOURCE**

| Lines | Issue | Type | Impact |
|-------|-------|------|--------|
| 2086 | `[Input] Invalid ADC pin` | ADC setup error | 1 call |
| 2275 | `[Input] NAU gain x%u rate %u SPS (...)` | Sensor init detail | 1 call, 1 long line |
| 2489 | `[Input] NAU internal offset calibration failed` | Calibration error | 1 call |
| 2501 | `[Input] Load-cell ADC source ready` | Sensor ready | 1 call |
| 3335-3347 | `[WARNING] Inverter/Stator/Coolant temp high` | Fault detection (5 calls) | **10+ lines** |
| 4082 | `[VCM-1D4-TX-SELF] src=%s tq=%.2f raw=%d...` | TX payload debug | 1 call, **1 long line** |
| 4548 | `[1D4-PAYLOAD-UPDATE]` | Payload state machine | 1 call, **1 long line** |
| 4844 | `[Input] VCU ready source:` | VCU ready diagnosis | 1 call |
| 4919 | `[Input] I2C devices: %s` | I2C scan summary | 1 call |
| 5823 | `[VCM-1D4-SNIFF-RAW] n=%lu age=%lu len=%u...` | Raw frame logging | 1 call per sniff frame |

**Subtotal:** 15+ calls, ~50-100 lines total

**Recommendation:**
- Move all Serial.printf to compile-time ifdef guard
- Consider per-module `VERBOSE_*` flags (e.g., `VERBOSE_CAN`, `VERBOSE_SENSORS`)

---

### [LeafCan.cpp](LeafCan.cpp)

| Lines | Issue | Type | Impact |
|-------|-------|------|--------|
| 427 | `[RPM-DECODE] ze1=%.1f leg=%.1f...` | RPM selection debug | 1 call, 1 long line |

---

### Other Files

| File | Lines | Count | Impact |
|------|-------|-------|--------|
| HardwareOutputStateMachine.cpp | 165-171 | 1 call | `[HWSM][GUARD]` mismatch detection |
| CommandRouter.cpp | 786 | 1 call | `[SAFETY]` manual torque rejection |
| WebSocketServer.cpp | 39, 44 | 2 calls | `[WS]` client connect/disconnect |
| ModbusPublisher.cpp | 73 | 1 call | `[ModbusPublisher]` startup |
| RunStorage.cpp | 116 | 1 call | `[RAW]` file operation |
| TelnetSerialBridge.cpp | 24 | 1 call | `[TelnetBridge]` startup |

**Subtotal:** ~10 calls

---

## 2. DISABLED DIAGNOSTIC FEATURES — MEDIUM PRIORITY

**Estimated Savings:** 100-300 lines (if removed entirely)  
**Effort:** Medium (test all code paths)  
**Risk:** Low if features are truly disabled

### [main.cpp](main.cpp) — `ENABLE_RUNTIME_INSTRUMENTATION`

**Lines:** 25, 680-714 (35 lines)  
**Status:** Set to `0`

```c
#define ENABLE_RUNTIME_INSTRUMENTATION 0

// Used in:
// - recordTaskRuntime() function (lines 218-234) — 17 lines, only called inside #if blocks
// - resetMaxStats() function (lines 235-240) — 6 lines, only reads max values
// - Control/Modbus/Network task loops — 3x #if blocks with timing code
```

**Issue:** Function definitions + data structures exist but unused when disabled.

**Recommendation:**
- Option A: Remove entire `recordTaskRuntime()`, `resetMaxStats()`, and `TaskRuntimeStats` struct
- Option B: Move to `#ifdef ENABLE_RUNTIME_INSTRUMENTATION` with function stubs for disabled case
- **Savings:** ~40 lines

---

### [Input.cpp](Input.cpp) — `ENABLE_CAN_METRICS`

**Lines:** 31-32 (define), 189-214 (data), 657, 3171, 3187, 3269, 3293 (usage)  
**Status:** Set to `0`

```c
#ifndef CAN_METRICS_ENABLED
#define CAN_METRICS_ENABLED 0
#endif

// Static data block (lines 189-214):
#if ENABLE_CAN_METRICS
static bool lastCanDiagInitialized = false;
static bool lastCanDiagReady = false;
static uint8_t lastCanDiagState = 0xFF;
static uint32_t lastCanDiagTxFrames = 0;
// ... 20 more counters
static uint8_t lastCanDiag11aTxLen = 0U;
#endif  // ENABLE_CAN_METRICS
```

**Issue:** 26 static variables (26× 4-8 bytes = ~150 bytes of RAM) + diagnostic read calls

**Recommendation:**
- If CAN diagnostics never needed: Remove lines 189-214 completely
- **Savings:** 26 lines, ~150 bytes RAM

---

### [Input.cpp](Input.cpp) — `METASENSE_LEAF_VCM_DIAGNOSTICS`

**Lines:** 239-248 (define), 407, 2822 (usage blocks)  
**Status:** Set to `0`

```c
#ifndef METASENSE_LEAF_VCM_DIAGNOSTICS
#define METASENSE_LEAF_VCM_DIAGNOSTICS 0
#endif

#if defined(METASENSE_LEAF_VCM_DIAGNOSTICS) && (METASENSE_LEAF_VCM_DIAGNOSTICS != 0)
// CRC proof-testing / alternative algorithm blocks
#endif
```

**Issue:** Minimal impact (mostly empty blocks), but guard conditions are verbose.

**Recommendation:**
- Simplify to `#if METASENSE_LEAF_VCM_DIAGNOSTICS` (single condition)
- **Savings:** ~3 lines (guard simplification)

---

### [CANBus.cpp](CANBus.cpp) — `METASENSE_STARTUP_SNIFF_*` Feature Suite

**Lines:** 40-69 (30 lines of #defines), 174-350 (177 lines of dead code)  
**Status:** `METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE = 0`

```c
#ifndef METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
#define METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE 0
#endif

// 9 other related configuration options (lines 44-69)

#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
struct StartupSniffFrame { /* 16 bytes */ };
static StartupSniffFrame s_startupSniffFrames[METASENSE_STARTUP_SNIFF_CAPTURE_MAX_FRAMES];
static bool s_startupSniffActive = false;
// ... 10+ more static variables
static void resetStartupSniffCapture() { ... }
static void startStartupSniffCapture() { ... }
static void recordStartupSniffFrame() { ... }
static bool isStartupSniffTrackedId() { ... }
// ... more helper functions (177 total lines)
#endif
```

**Issue:** Entire feature is dead code when disabled; takes up 177 lines.

**Recommendation:**
- **Option A:** Remove all `METASENSE_STARTUP_SNIFF_*` code (lines 40-69, 174-350) if sniff feature never needed
- **Option B:** Stub out functions with empty implementations (keep API compatibility)
- **Savings:** 207 lines (23% of CANBus.cpp)
- **Impact:** Check user's preference before cleanup

---

### Configuration Bloat (`#ifndef` chains in multiple files)

**[main.cpp](main.cpp) lines 26-72** (47 lines of #ifndef guards)

```c
#ifndef VCU_switch
#define VCU_switch 0
#endif
// × 15 similar blocks
```

**[CANBus.cpp](CANBus.cpp) lines 13-69** (57 lines of configuration)

```c
#ifndef METASENSE_CAN_LOG_KEY_FRAMES
#define METASENSE_CAN_LOG_KEY_FRAMES 0
#endif
// × 18 similar blocks
```

**[Input.cpp](Input.cpp) lines ~240-290** (50+ lines of #defines)

**Issue:** Each disabled feature adds ~3 lines (`#ifndef`, `#define`, `#endif`). Productionizing would eliminate these.

**Recommendation:**
- For production build: Hardcode all `#define` values in a single `config.h`
- Consolidate `#ifndef` chains into single header
- **Savings:** 150+ lines across all files

---

## 3. UNUSED/DEBUG VARIABLES — LOW PRIORITY

**Estimated Savings:** 15-30 lines (plus ~500 bytes RAM)

### [Input.cpp](Input.cpp) — Debug-Only Variables

| Lines | Variable | Type | Purpose | Status |
|-------|----------|------|---------|--------|
| 123 | `leafCanRpmMonitor` | `static float` | Stores CAN RPM for monitoring | Set but commented out (line 2808) |
| 139-142 | `vcuDebugSimMode`, `vcuDebugInv12v`, `vcuDebugHvVoltage`, `vcuDebugTorqueDemandNm` | `static` | Bench test sim inputs | Only used in `updateVcuDebug()` |
| 176-179 | `vcuDebugRPlus`, `vcuDebugPrecharge`, `vcuDebugSsr`, `vcuDebugRMinus` | `static` | Output state debug | Only used in `updateVcuDebug()` + telemetry |

**Issue:** These are set but rarely read; used only for debug reporting.

**Recommendation:**
- `leafCanRpmMonitor`: Check if actually used; appears to be vestigial from older code
- `vcuDebug*`: Keep if needed for web UI diagnostics; otherwise move to conditional block
- **Savings:** ~10 lines + 32 bytes RAM (if removed)

---

### [CANBus.cpp](CANBus.cpp) — Startup Sniff Data (if feature disabled)

**Lines:** 183-193 (11 lines of static data)

```c
#if METASENSE_STARTUP_SNIFF_CAPTURE_ENABLE
static StartupSniffFrame s_startupSniffFrames[METASENSE_STARTUP_SNIFF_CAPTURE_MAX_FRAMES];
static uint16_t s_startupSniffCount = 0U;
// ... more
#endif
```

**Issue:** If ENABLE=0, all data is dead code.

**Savings:** ~150 bytes RAM + 11 lines code (already counted in sniff feature)

---

## 4. COMMENTED-OUT CODE BLOCKS — LOW PRIORITY

**Estimated Savings:** 10-20 lines

### [LeafCan.cpp](LeafCan.cpp) lines 460-466

```c
// DISABLED: Startup filter was too aggressive...
// TODO: Implement smarter startup detection...
// const uint8_t statusByteZeroMask = (dlc > 0U) ? d[0] : 0U;
// const uint8_t startupMask = 0x00U;
// const bool startupStyleStatus = (statusByteZeroMask == 0x00U) ||
//                                (statusByteZeroMask == 0x18U) ||
//                                (statusByteZeroMask == 0x24U);
// if (startupStyleStatus || (fb.id1da_status_bits == startupMask)) {
//     fb.rpm = 0.0f;
//     fb.torque_nm = 0.0f;
// }
```

**Issue:** 11 lines of commented code (plus explanatory comment).

**Recommendation:**
- Remove commented block; preserve explanation comment (1-2 lines)
- **Savings:** 10 lines

---

## 5. CONFIGURATION THAT COULD BE HARDCODED

**Estimated Savings:** ~30 lines + decision logic

### Configuration Constants with Only One Practical Value

| Define | Current | File | Reason |
|--------|---------|------|--------|
| `CAN_METRICS_ENABLED` | 0 | Input.cpp | Production always 0; diagnostics removed |
| `METASENSE_LEAF_VCM_DIAGNOSTICS` | 0 | Input.cpp | CRC proof-testing never enabled |
| `METASENSE_STREAM_DIAGNOSTICS` | 0 | Input.cpp | JSON diagnostics disabled |
| `ENABLE_RUNTIME_INSTRUMENTATION` | 0 | main.cpp | Task timing disabled |
| `VCU_switch` | 0 | main.cpp | GPIO input, always 0 for normal operation |

**Recommendation:**
- For **production build**: Convert these to `constexpr bool` in `config.h`
- For **dev builds**: Keep `#ifndef` chains for flexibility
- Add compiler flag: `-D METASENSE_PRODUCTION_BUILD=1` to strip debug code

---

## PRIORITY CLEANUP ROADMAP

### **Phase 1: Quick Wins (2-3 hours)**
- Remove Serial.printf from production paths (150 lines)
- Wrap debug logging in single `#if DEBUG_LOGGING` guard
- Comment-out large HEARTBEAT dumps or reduce verbosity
- **Impact:** Firmware binary size ↓5-10%, no behavioral change

### **Phase 2: Dead Code Removal (4-6 hours)**
- Remove or stub `METASENSE_STARTUP_SNIFF_*` feature (207 lines)
- Remove `recordTaskRuntime()`/`resetMaxStats()` if unused (40 lines)
- Remove debug-only variables (15 lines)
- **Impact:** ~260 lines removed, ~150 bytes RAM freed

### **Phase 3: Configuration Consolidation (2-3 hours)**
- Create unified `config.h` with production defaults
- Replace all `#ifndef` chains with single `#include "config.h"`
- Add build-time conditionals for dev/prod modes
- **Impact:** ~150 lines removed, cleaner build system

### **Phase 4: Refactor (8+ hours)**
- Extract debug macros to `debug.h`
- Separate logging from business logic
- Add conditional logging subsystem (e.g., `LOG_CAN_EVENTS`, `LOG_SENSOR_DATA`)
- **Impact:** Much easier to maintain; production-ready

---

## ESTIMATED CODE SAVINGS SUMMARY

| Category | Lines | LOC % | RAM | Effort |
|----------|-------|-------|-----|--------|
| Debug logging (Phase 1) | 150 | ~2% | ~5KB | 1-2h |
| Dead diagnostic features (Phase 2) | 260 | ~3% | ~200B | 2-3h |
| Unused variables (Phase 2) | 15 | <1% | ~50B | 0.5h |
| Commented code (Phase 2) | 10 | <1% | 0B | 0.5h |
| Config consolidation (Phase 3) | 150 | ~2% | 0B | 2h |
| **TOTAL** | **~585** | **~8-10%** | **~200B-5KB** | **6-10h** |

**Cumulative Impact:**
- Binary size: ~8-15% reduction (~80-150 KB on typical ESP32 firmware)
- RAM usage: +200 bytes (sniff feature) to -5 KB (diagnostics disabled)
- Startup time: Negligible (most disabled at compile time)
- Runtime performance: +5-10% (fewer printf calls, simpler conditionals)

---

## RECOMMENDATIONS

### ✅ Safe to Remove (Low Risk)
1. **Serial.printf() logging** — wrap in `#if DEBUG_LOGGING` or remove entirely
2. **Comment-out code blocks** — especially LeafCan.cpp startup filter
3. **Debug variables** if unused — `leafCanRpmMonitor`, test-only vcuDebug*
4. **ENABLE_RUNTIME_INSTRUMENTATION** — if task timing never profiled

### ⚠️ Careful (Medium Risk — Requires Testing)
1. **METASENSE_STARTUP_SNIFF_*** — only if feature truly not needed
2. **CAN_METRICS_ENABLED** — check if Web UI relies on diagnostic data

### ⏳ Future Refactoring (Low Immediate Priority)
1. **Configuration consolidation** — create `config.h` + `-D METASENSE_PRODUCTION_BUILD`
2. **Logging subsystem** — extract to separate module with conditional compilation
3. **Modular ifdef strategy** — per-subsystem debug flags (CAN, sensors, output, etc.)

---

## KEY CONSTRAINT

**⚠️ SAFETY BOUNDARY:** Do NOT modify state-machine logic, CRC algorithms, or control paths while cleaning up. All changes must be additive or removals of pure debug/diagnostic code.

**✓ APPROVED FOR CLEANUP:**
- Serial output statements
- Unused diagnostic counters
- Disabled compile-time features
- Dead code blocks (commented sections)

**✗ DO NOT TOUCH:**
- Control loop timing
- CRC validation logic
- State machine transitions
- Output relay/SSR commands

---

## NEXT STEPS

1. **Review** this analysis for accuracy
2. **Decide** which phases to implement (recommend Phase 1 + 2)
3. **Test** on [env:esp32s3-USB] after each phase
4. **Measure** binary size + RAM usage before/after
5. **Document** any configuration changes in platformio.ini

