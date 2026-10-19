# MetaSense-DYNO Firmware: AI Agent Guidelines

## Quick Orientation

**MetaSense-DYNO** is an ESP32-S3 firmware for a **Nissan Leaf electric dyno testing platform**. It controls a Leaf inverter via CAN bus, acquires multi-sensor telemetry (RPM, torque, EGT, temperatures), runs a real-time PI RPM feedback loop, and publishes data via WebSocket to a web UI.

- **Primary language**: C++ (Arduino-compatible, FreeRTOS-based)
- **Build system**: PlatformIO
- **Key hardware**: ESP32-S3, Nissan Leaf Thunderstruck inverter (CAN), NAU7802 load-cell (I2C), MCP9600 EGT thermocouple, BME680 environmental
- **Development flow**: Code → Build → Upload (USB or OTA) → Monitor via Serial/Telnet/WebSocket

---

## 🎯 Before You Start: Critical Constraints

### ⚠️ HARD INVARIANTS (Breaking these = Silent Failures or Safety Hazards)

1. **CRC Algorithm Source-of-Truth**
   - **ONLY** the global CRC8-MSB function in `src/LeafCan.cpp` (declared in `include/LeafCrc.h`) is authoritative.
   - ❌ **NO** learned residue tables, alternate helper functions, diagnostic counters, or fallback CRC logic anywhere in TX/RX paths.
   - **Consequence**: Nissan inverter rejects frames with wrong CRC; motor stops responding with "100 000" fault.
   - **Algorithms** (✅ validated on real Thunderstruck frames):
     - **0x1DA RX**: CRC8-MSB(poly=0x85, init=0x00, xorOut=0xBF) over `[0xDA + b0..b6]`, then XOR by HCM_CLOCK residue.
     - **0x1D4 TX**: CRC8-MSB(poly=0x85, init=0x00, xorOut=0xFF) over `[b0..b6]` (NO frame ID prepended), final calc applies NO xorOut.
     - **0x11A keep-alive**: No CRC (static/mux-only).
   - See [docs/CRC_ALGORITHM_DEFINITIVE.md](docs/CRC_ALGORITHM_DEFINITIVE.md) for complete spec + reference frames.

2. **Precharge Dwell Contract**
   - **Must wait** until HV ≥ 300V **AND** elapsed ≥ 2500ms before leaving `INIT` state.
   - State machine allows `prechargeCompleted` only when BOTH gates are satisfied.
   - ❌ **DO NOT** auto-complete on precharge relay dropout, state-leave, or timeout exception.
   - **Consequence**: Nissan inverter faults if HV rises too fast; relay dropout damages HV capacitors.

3. **CAN RX Freshness > PI Loop**
   - Control loop assumes 0x1DA frames arrive **within 30ms** (`METASENSE_CAN_RX_TIMEOUT_MS`).
   - **On timeout**: state machine forces FAULT (inverter assumed dead).
   - ❌ **DO NOT** cache stale feedback across loop iterations without age-check.
   - **Consequence**: Stale RPM + old torque command = runaway motor.

4. **0x1D4 TX Cadence = 10ms (Fixed)**
   - Paired 0x11A keep-alive **must** follow 0x1D4 at 10ms rate (never skip, never batch).
   - ❌ **DO NOT** reduce cadence or add back-pressure gates (e.g., "skip if backlog > N").
   - **Consequence**: Inverter interprets gap > 50ms as communication loss (pre-faults).

5. **Throttle Output (GPIO47) is NOT Gated by VCU Ready**
   - Engine throttle servo responds **directly** to Pot2 ADC (GPIO2), NOT by state machine.
   - ❌ **DO NOT** add VCU-ready gate to throttle output.
   - **Consequence**: Test workflows need manual throttle while waiting for CAN handshake.

6. **State-Machine Logic is Approval-Gated**
   - Relay transitions, precharge dwell, fault latches are safety-critical.
   - ❌ **DO NOT** refactor `HardwareOutputStateMachine.cpp` state logic without explicit user approval.
   - **Review checklist**: Verify state table (lines ~50–250), ensure break-before-make timing, confirm precharge contract.

---

## 📁 Project Structure

### Key Directories & Files

```
platformio.ini              Build config (2 main envs: esp32s3-USB, esp32s3-ota)
include/
  ├─ globals.h             GPIO pin defines (compile-time safety checks)
  ├─ CanConfig.h           CAN/telemetry compile flags
  ├─ LeafCan.h / LeafCrc.h  Nissan frame definitions + CRC authority
  └─ MetaSenseDynoVCU.h    Global config

src/
  ├─ main.cpp              Task setup, heartbeat, OTA, WiFi
  ├─ Input.cpp / Input.h    🔴 LARGE (~3000 lines): sensor acquisition, JSON telemetry publish
  ├─ controlTask.cpp        PI loop, DynoStateMachine, HardwareOutputStateMachine
  ├─ CANBus.cpp / LeafCan.cpp
                            TWAI polling, frame decode/encode, CRC validation
  ├─ HardwareOutputStateMachine.cpp 🔴 SAFETY-CRITICAL
                            Relay FSM (INIT→IDLE↔MOTOR/DYNO+FAULT)
  ├─ DynoStateMachine.cpp   Recording, auto-ramp profiles
  ├─ Settings.cpp           NVS (flash) persistence for tuning params
  ├─ WebSocketServer.cpp    HTTP/WebSocket handlers
  ├─ PIController.cpp       Discrete PI at 10ms (Kp=0.073, Ki=0.524)
  └─ ...

lib/
  ├─ HAL_CAN/              TWAI abstraction
  ├─ HAL_PWM/              PWM drivers (throttle, dyno brake servo)
  ├─ HAL_RPM/              Tachometer input (ADC + fallback CAN)
  ├─ HAL_Temp/             MCP9600 thermocouple EGT
  └─ ...

data/
  ├─ index.html            Live dashboard (rpm, torque, power, relay states, CAN metrics)
  ├─ settings.html         GUI for PI tuning, load-cell calibration, motor RPM cap
  ├─ trend.html            30-min rolling history
  └─ ...

docs/
  ├─ CRC_ALGORITHM_DEFINITIVE.md         ⭐ CRC reference (algorithms, pseudocode, test frames)
  ├─ NISSAN_LEAF_1DA_CRC.md              0x1DA reverse-engineering details
  └─ CRC_ALGORITHM_REPORT_2026-08-15.md  Investigation history

README.md                  Quick start, CRC summary, runtime timings, hardware state machine
PI_TUNING_CALCULATION.md   PI pole-placement derivation
DynoModbus mapping.txt     Modbus register layout
Leaf-CAN_ZE1.dbc          CAN database (frame IDs, signals, scaling factors)
```

### Repository Memory Files

Located in `/memories/repo/`:
- **platformio-notes.md** → Build configs, ADC pin mapping, CAN behavior, WiFi workarounds
- **can-monitor-mode-architecture.md** → Telemetry filtering (Mode 0 = core only, Mode 1 = full CAN metrics)
- **PI_TUNING_2026-08-27.md** → Live tuning reference (anti-windup, settling time, pot tuning)
- **runtime-instrumentation.md** → Task load measurement (enable via `ENABLE_RUNTIME_INSTRUMENTATION=1`)
- **compile-time-diagnostics.md** → Diagnostic flags reference
- **leaf-crc.md** → CRC residue tables for HCM_CLOCK (c0=0xC0, c1=0x67, c2=0x0B, c3=0xAC)

---

## 🔨 Build & Development Workflow

### PlatformIO Environments

| Environment | Use Case | Upload Method |
|---|---|---|
| `esp32s3-USB` | Local development (default) | esptool over COM3 |
| `esp32s3-ota` | Remote testing on network | ESPota to 192.168.0.211 |

**Build flags** (CanConfig.h + platformio.ini):
- `METASENSE_LEAF_CAN_TX_ENABLED=1` → Allow CAN TX (0x1D4, 0x11A)
- `METASENSE_CAN_MONITOR_MODE=0` → Core telemetry only (1 = full CAN metrics in WebSocket)
- `METASENSE_VCU_SIM_MODE=0` → Use real hardware (1 = bench sim, no inverter)

### VS Code Workspace Tasks

Available tasks (run via **Terminal → Run Task**):
- **Build (esp32s3-USB)** → `platformio run -e esp32s3-USB`
- **Upload (esp32s3-USB)** → `platformio run -e esp32s3-USB -t upload` ⚠️ **Close monitor/release COM3 before upload**
- **Upload LittleFS (esp32s3-USB)** → Upload web UI assets to LittleFS partition
- OTA equivalents for `esp32s3-ota`

**Preferred workflow** (per user memory): Use VS Code tasks instead of one-liners; better output feedback.

### Common Build Metrics

- **Flash usage**: ~46% (~1.1 MB / 2.4 MB app partition)
- **RAM usage**: ~18% (~296 KB / 1.6 MB)
- **Build time**: ~15–20s (clean) on typical dev machine

---

## 🏗️ Architecture Overview

### Task-Based Multi-Core Layout (FreeRTOS on ESP32-S3)

| Task | Core | Period | Priority | Main Responsibilities |
|------|------|--------|----------|---|
| **Control** | 1 | 10ms (100 Hz) | 5 (high) | RPM PI loop, dyno FSM, relay commands, sensor telemetry aggregation |
| **Network** | 0 | 25ms (40 Hz) | 3 (medium) | WiFi, WebSocket publish, OTA, command routing |
| **Modbus** | 0 | 50ms (20 Hz) | 1 (low) | Register publishing to external Modbus clients |

**Core principle**: Control logic isolated from networking. All HV relay commands gated through state machines.

### Signal Flow: Sensor → PI Loop → CAN TX → Inverter

```
Input Sensors (10ms cadence in controlTask)
  ├─ RPM (Tachogen ADC primary, CAN motor RPM fallback)
  ├─ Torque (NAU7802 load-cell, I2C @ 80 SPS, EMA filtered)
  ├─ Temps (MCP9600 EGT, BME680 ambient, Leaf CAN 0x55A)
  ├─ HV voltage, inverter fault bits (0x1DA CAN RX)
  └─ Potentiometer setpoints

    ↓ 

PI RPM Feedback Loop (10ms, ~1.5s settling)
  Compares: RPM_actual vs RPM_setpoint
  Output: torque_demand (-50 to +50 Nm)

    ↓

HardwareOutputStateMachine
  ├─ State: INIT → IDLE ⇄ MOTOR/DYNO + FAULT
  ├─ Gates: precharge dwell, CAN freshness, inverter ready
  └─ Outputs: RB+, RB-, SSR, precharge relays (GPIO)

    ↓

CAN TX (10ms cadence, fixed)
  ├─ 0x1D4: torque_demand, relay bits, CRC8
  └─ 0x11A: keep-alive, mux pattern

    ↓

Nissan Leaf Inverter
  Responds with 0x1DA feedback (motor status, RPM, temps, fault bits)
  Closes the loop!
```

### Key Components

#### **Input::loop() (3000+ lines)**
- Acquires all sensors (ADC, I2C, CAN RX).
- Computes telemetry JSON.
- Publishes to WebSocket every 16–50ms.
- **Hotspot**: Load-cell filtering, JSON serialization.

#### **CANBus::poll() (10ms cadence)**
- Drains 64 TWAI frames per call.
- Decodes 0x1DA (motor feedback), 0x55A (temps), 0x11A (keep-alive rx).
- Validates CRC8 on 0x1DA against HCM_CLOCK residue.
- **Hotspot**: CRC8 operations, frame age tracking.

#### **HardwareOutputStateMachine::update() (Safety-Critical)**
- FSM managing 4 HV relays: RB+, RB-, SSR (load contactor), precharge.
- Enforces: break-before-make (20ms gap), precharge dwell (2500ms + HV300V), CAN freshness (30ms timeout).
- Drives GPIO relay outputs **synchronously** (no queue).
- **State transition debounce**: 50ms (candidate state must be stable).

#### **PIController (Discrete 10ms)**
- Kp=0.073, Ki=0.524 (derived from pole-placement, ζ=0.7, ωn=4 rad/s).
- Integral anti-windup: clamped to output range [-50, +50] Nm.
- Setpoint source: ADC pot (GPIO3) or UI slider.
- Live Kp tuning via Pot3 (GPIO6) range 0.005–0.200.

#### **Settings Persistence (NVS)**
- 30+ tunable parameters (PI gains, load-cell calibration, dyno inertia, motor RPM cap, filter modes).
- Read/write at startup and on UI `/settings/update` POST.
- Survives reboots.

---

## 🧪 Testing & Validation

### Test Modes

| Mode | Flag | Use Case |
|------|------|----------|
| **Real Inverter** | `METASENSE_LEAF_CAN_TX_ENABLED=1`, `METASENSE_VCU_SIM_MODE=0` | Production, real Leaf inverter on CAN bus |
| **Bench Sim** | `METASENSE_VCU_SIM_MODE=1` | GUI dev, state machine logic (no HW) |
| **Passive RX** | `METASENSE_LEAF_CAN_LISTEN_ONLY=1`, TX disabled | Sniff inverter traffic (no TX) |

### Validation Checkpoints

1. **CRC Algorithms**: See [docs/CRC_ALGORITHM_DEFINITIVE.md](docs/CRC_ALGORITHM_DEFINITIVE.md); all 3 algorithms validated on real Thunderstruck frames.
2. **PI Tuning**: Kp/Ki tested on bench; ~1.5s settling to RPM target. Live tuning via pot; historical values in PI_TUNING_2026-08-27.md.
3. **Precharge**: 2500ms dwell + HV ramp validated via manual relay toggle tests.
4. **WebSocket Telemetry**: Monitor via `/index.html` live dashboard; enable `METASENSE_STREAM_DIAGNOSTICS=1` to measure backlog/drop counts.
5. **Raw Data Capture**: WebSocket cmd `START_RAW_CAPTURE` → CSV export to LittleFS; browse via `/captures` endpoint.
6. **Task Instrumentation**: Enable `ENABLE_RUNTIME_INSTRUMENTATION=1` (main.cpp line 24); logs `[RUNTIME]` every 10s with per-task CPU load %.

---

## ⚠️ Common Pitfalls & Workarounds

### Code Hotspots

| File | Issue | Impact |
|------|-------|--------|
| **Input.cpp** (3000+ lines) | Largest file; load-cell/CAN/JSON paths intertwined | Touch carefully; measure EMA calc, JSON serialize, WebSocket transmit |
| **CANBus.cpp** | 64-frame drain per 10ms call; CRC8 ops here | Overload blocks PI loop immediately |
| **HardwareOutputStateMachine.cpp** | Relay GPIO writes synchronous, break-before-make critical | Refactoring requires state-table review + testing |

### Known Workarounds

| Problem | Workaround | Status |
|---|---|---|
| WiFi drops every 5–10 min under load | `WiFi.setSleep(false)`, `setTxPower(19.5dBm)`, `setAutoReconnect(true)` | ✅ Applied in main.cpp |
| TWAI listen-only blocks RX of self-transmitted IDs | Use NORMAL mode + app-level TX disable OR sniff env | ✅ Env-based config |
| MCP9600 repeated-start I2C unreliable | Split writes: write → STOP → read | ✅ TempHAL.cpp |
| NAU7802 noisy at 320 SPS | Keep runtime at 80 SPS; use Hampel outlier filter | ✅ Stable |
| WebSocket backlog blocks telemetry | Drop slow clients: only enqueue if `queueLen() == 0` | ✅ Input.cpp ~1460 |

### Testing Blind Spots

- No CAN unit tests (hardware-dependent; sniff env exists but no CI).
- No load-cell stress tests (bare device, no automated calibration suite).
- No WiFi reconnect tests (requires real AP + network loop).
- No state-machine formal verification (FSM validated by inspection only).

---

## 📝 Code Conventions

### Naming

| Category | Pattern | Example |
|---|---|---|
| Constants | `k<PascalCase>` global, `METASENSE_<UPPER_SNAKE>` compile-time | `kControlPeriodMs`, `METASENSE_LEAF_CAN_TX_ENABLED` |
| Member vars | `<snake_case>_` private (trailing underscore) | `lastUpdateMs_`, `piIntegrator_` |
| Struct members | `<snake_case>` (no underscore) | `LeafInvFeedback::rpm`, `feedback_.rpm` |
| Bool functions | `is<Action>()`, `get<Property>()`, `has<State>()` | `isVcuReady()`, `isRbPlusActive()` |
| Getters/Setters | `get<Name>()` / `set<Name>()` | `getCalibrationFactor()` |
| Init/Loop | `begin()` setup, `loop()` update | `Input::begin()`, `Input::loop()` |

### Namespaces

- `MetaSense::<subsystem>::function()` — e.g., `MetaSense::Input::loop()`, `MetaSense::CANBus::poll()`

### Mutual Exclusion & Timing

- **I2C**: All access gated by `MetaSense::I2cBus::take()` / `give()` mutex (50ms timeout).
- **Timestamping**: Both `millis()` (global clock) and `micros()` (runtime precision) used; loops measure elapsed time for fixed-period dispatch.
- **Frame age tracking**: Every CAN frame tagged with arrival time; loss > 30ms triggers fault.

---

## 🚀 Quick Start for AI Agents

### First Time Editing This Project?

1. **Read this file** (you are here!) to understand hard invariants and architecture.
2. **Check existing docs**:
   - [README.md](README.md) → CRC algorithms, runtime timings, hardware state machine.
   - [docs/CRC_ALGORITHM_DEFINITIVE.md](docs/CRC_ALGORITHM_DEFINITIVE.md) → CRC reference (algorithms, test frames).
3. **For CAN/CRC work**: Verify your change references ONLY `src/LeafCan.cpp` CRC authority; NO fallbacks.
4. **For state machine work**: Get explicit user approval before refactoring `HardwareOutputStateMachine.cpp`.
5. **For new features**: Check `/memories/repo/` for conventions, build configs, and task instrumentation setup.

### Common Tasks

**Debug CAN traffic:**
- Enable `METASENSE_CAN_MONITOR_MODE=1` → UI shows all 0x1DA/0x1D4/0x11A fields + CRC validity.
- Enable `METASENSE_LEAF_VCM_DIAGNOSTICS=1` → Serial logs `[VCM-...]` frame details.

**Profile task CPU load:**
- Set `ENABLE_RUNTIME_INSTRUMENTATION=1` in main.cpp line 24.
- Every 10s, serial logs `[RUNTIME]` with per-task load % and overrun count.

**Tune PI gains live:**
- Rotate Pot3 (GPIO6) to adjust Kp in real-time (0.005–0.200 range).
- Or use `/settings.html` UI slider; changes take effect immediately.

**Capture raw sensor data:**
- WebSocket command: `START_RAW_CAPTURE` → writes `/captures/raw_<epoch>.csv` to LittleFS.
- Configurable 60s window; browse captures via `/captures` HTTP endpoint.

**Validate CRC implementation:**
- See [docs/CRC_ALGORITHM_DEFINITIVE.md](docs/CRC_ALGORITHM_DEFINITIVE.md) for reference frames.
- Build test harness in `src/` or use CI if available.

---

## 📞 Questions Before Making Changes?

- **State-machine refactoring?** → Ask user for approval; review HardwareOutputStateMachine.cpp state table (lines ~50–250) first.
- **CRC algorithm changes?** → Verify change is in `src/LeafCan.cpp` ONLY; check poly, init, xorOut against README + docs.
- **WebSocket telemetry frequency?** → Default 50ms; configurable via `METASENSE_WS_FAST_PERIOD_MS` (InputConfig.h).
- **Task timing issues?** → Enable `ENABLE_RUNTIME_INSTRUMENTATION=1` to measure CPU load; check for blocking I2C or JSON operations.

---

## 📚 Related Links

- [README.md](README.md) — Quick start, CRC summary, runtime timings.
- [docs/CRC_ALGORITHM_DEFINITIVE.md](docs/CRC_ALGORITHM_DEFINITIVE.md) — CRC algorithms, pseudocode, reference frames.
- [PI_TUNING_CALCULATION.md](PI_TUNING_CALCULATION.md) — PI pole-placement derivation (ζ=0.7, ωn=4 rad/s).
- [Leaf-CAN_ZE1.dbc](Leaf-CAN_ZE1.dbc) — CAN frame/signal definitions.
- [platformio.ini](platformio.ini) — Build configs, environment definitions.
- [include/CanConfig.h](include/CanConfig.h) — Compile-time diagnostic flags.
