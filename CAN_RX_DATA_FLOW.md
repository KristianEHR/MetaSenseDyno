# Complete CAN RX Data Flow: Raw Frame to WebSocket Telemetry

## Overview
This document traces the complete data flow from raw CAN frame reception through to WebSocket telemetry output for the Nissan Leaf inverter telemetry system.

---

## 1. RAW FRAME RECEPTION

### Stage: CANBus::poll()
**File:** [src/CANBus.cpp](src/CANBus.cpp#L880)  
**Function:** `void CANBus::poll(uint32_t nowMs)` (lines 880-1005)  
**Purpose:** Continuously receives and validates raw CAN frames from TWAI controller

#### Process:
1. **Query TWAI Status** (line 885)
   - Get `twai_status_info_t` from hardware controller
   - Check for BUS_OFF, STOPPED, or error conditions
   
2. **Receive Raw Frames** (line 930)
   - Loop: `s_canHal.receive(id, data, len, isExtended)`
   - Extract: CAN ID, 8-byte payload, data length code
   
3. **Validate Frame** (lines 940-950)
   - Skip excluded IDs (0x120, 0x11A, 0x1D4 unless sniffing)
   - **CRC Gate for 0x1DA:** Only frames with valid CRC are decoded
   - Store raw payload in `s_stats.last1daData[8]` for diagnostics

4. **Route to Decoder** (line 955-960)
   - If ID is accepted (0x1DA, 0x1DC, 0x55A), call `LeafCan::decodeFrame()`
   - Pass: `twai_message_t`, `LeafInvFeedback&`, `now_ms`

---

## 2. FRAME DECODING

### Stage: LeafCan::decodeFrame()
**File:** [src/LeafCan.cpp](src/LeafCan.cpp#L395)  
**Function:** `void LeafCan::decodeFrame(const twai_message_t &msg, LeafInvFeedback &fb, uint32_t now_ms)` (lines 395-500+)  
**Purpose:** Extract fields from raw 8-byte CAN payload and write to LeafInvFeedback struct

#### Structure Definition
**File:** [include/LeafCan.h](include/LeafCan.h#L12)
```cpp
struct LeafInvFeedback {
    // Primary feedback signals
    float rpm;                      // Motor speed from 0x1DA [rpm]
    float torque_nm;                // Actual torque from 0x1DA [Nm]
    float input_voltage;            // MG input voltage from 0x1DA [V]
    float inverter_temp;            // IGBT temp from 0x1DC/0x55A [°C]
    float stator_temp;              // Winding temp from 0x1DC/0x55A [°C]
    float coolant_temp;             // Coolant temp from 0x1DC/0x55A [°C]
    
    // Status flags
    bool ready;                     // Inverter ready state
    bool fault;                     // Fault condition
    bool warning;                   // Warning condition
    bool limp;                      // Limp mode
    
    // Timestamps (ms since system boot)
    uint32_t rpm_update_ms;         // Last valid RPM frame timestamp
    uint32_t torque_update_ms;      // Last valid torque frame timestamp
    uint32_t temps_update_ms;       // Last valid temperature frame timestamp
    uint32_t status_update_ms;      // Last valid status frame timestamp
    uint32_t last_update_ms;        // Most recent any valid frame
    
    // Decode counters (diagnostics)
    uint32_t rpm_frames;            // Count of 0x1DA RPM frames
    uint32_t torque_frames;         // Count of 0x1DA torque frames
    uint32_t temps_frames;          // Count of temp frames (1DC+55A)
    uint32_t status_frames;         // Count of status frames
    
    // Raw candidates for debugging
    float id1da_ze1_rpm;            // ZE1 Motorola-decoded RPM candidate
    float id1da_leg_rpm;            // Legacy LE word RPM candidate
    float id1da_ze1_tq;             // ZE1 Motorola torque candidate [Nm]
    float id1da_leg_tq;             // Legacy LE word torque candidate [Nm]
    
    // CRC & status diagnostics
    uint8_t mg_clock;               // HCM clock phase (0x1DA byte 7, bits 6:4)
    uint8_t mg_error_codes;         // Motor error code (0x1DA byte 7, bits 3:0)
    uint8_t crc_1da;                // CRC byte from 0x1DA frame
    uint8_t inv_status_bit;         // Inverter status bit
    uint8_t inv_fault_map;          // Fault bit mapping
    // ... other diagnostics fields
};
```

#### Decode Path by CAN ID

##### **0x1DA (MotorSpeed frame)**
**Decode functions called:**
- `decodeInputVoltage(d)` → `fb.input_voltage` [V]
- `decodeZe1OutputRevolution(d)` → candidate ZE1 RPM
- `decodeMotorSpeed(d)` → candidate legacy RPM
- `selectRpmCandidate(ze1, legacy)` → **`fb.rpm`** [rpm] ✓ Output
- `decodeZe1Clock(d)` → `fb.mg_clock` (bits 6:4 of byte 7)
- `decodeZe1ErrorCodes(d)` → `fb.mg_error_codes` (bits 3:0 of byte 7)
- `decodeZe1Crc(d)` → `fb.crc_1da`
- If len >= 8:
  - `decodeZe1TorqueNm(d)` → candidate ZE1 torque
  - `decodeMotorTorque(d)` → candidate legacy torque
  - **`fb.torque_nm`** = (ZE1 valid?) ? ZE1 : legacy [Nm] ✓ Output
  - Update `fb.torque_update_ms`, `fb.torque_frames`

##### **0x1DC (Temperature frame)**
- Extract: inverter_temp, stator_temp, coolant_temp
- Update: `fb.temps_update_ms`, `fb.temps_frames`, `fb.temps_1dc_frames`

##### **0x55A (Alternate Temperature frame)**
- Extract: explicit temperature channels (motor, com board, IGBT, driver)
- Map to: `fb.inverter_temp`, `fb.stator_temp`, `fb.coolant_temp`
- Update: `fb.temps_55a_frames`

#### Timestamp Updates
Each decode path updates:
- `fb.last_update_ms = now_ms` (every valid frame)
- `fb.{rpm|torque|temps|status}_update_ms = now_ms` (per field)

---

## 3. STORAGE IN CANBUS SINGLETON

### Global Storage
**File:** [src/CANBus.cpp](src/CANBus.cpp#L121)
```cpp
// Line 121: Static module-level storage
LeafInvFeedback s_feedback{};  // Initialized to zeros, then populated by decode loop
```

### Access Function
**File:** [include/CANBus.h](include/CANBus.h#L114) / [src/CANBus.cpp](src/CANBus.cpp#L1058)
```cpp
const LeafInvFeedback& feedback() {
    return s_feedback;  // Returns const reference to decoded data
}
```

---

## 4. TELEMETRY PUBLISHING

### Stage 1: Poll CAN Frames
**File:** [src/Input.cpp](src/Input.cpp#L2736)  
**Function:** `void pollLeafCanFrames(uint32_t nowMs)` (lines 2736-2810)  
**Purpose:** Fetch latest feedback and route to telemetry update

#### Key Operations:
1. **Initialize CAN** (line 2741)
   - `MetaSense::CANBus::configure(kLeafCanConfig)` (once)
   - `MetaSense::CANBus::poll(nowMs)` (every cycle)

2. **Get Latest Feedback** (line 2769)
   ```cpp
   const LeafInvFeedback& leafFb = MetaSense::CANBus::feedback();
   ```

3. **Validate Freshness** (line 2771-2774)
   - Check: `leafFb.rpm_update_ms != 0`
   - Check: `(now - leafFb.rpm_update_ms) < CAN_TEMP_TIMEOUT_MS` (typically 500ms)
   - If not fresh → use default/cached value

4. **Update Telemetry Fields** (lines 2780-2790)
   - RPM: Copy `leafFb.rpm` → telemetry cache
   - Torque: Copy `leafFb.torque_nm` → telemetry cache
   - Temps: Copy `leafFb.{inverter_temp, stator_temp, coolant_temp}` → telemetry cache

### Stage 2: Build Telemetry Structure
**File:** [src/Input.cpp](src/Input.cpp#L4770-4830)  
**Called from:** `computeAndPublishTelemetry()` and `publishTelemetry()`

#### Extract from LeafInvFeedback → Telemetry
The system reads the global `s_feedback` multiple times per telemetry cycle:

```
STORAGE LAYER                    TELEMETRY LAYER
─────────────────────            ─────────────────────
LeafInvFeedback s_feedback       MetaSense::Telemetry
│                                │
├─ .rpm ────────────────────────────> .leaf_rpm [float]
├─ .torque_nm ──────────────────────> .leaf_torqueNm [float]
├─ .input_voltage ──────────────────> .vcuHvVoltage [float]
├─ .inverter_temp ──────────────────> .leaf_invTempC [float]
├─ .stator_temp ────────────────────> .leaf_statorTempC [float]
├─ .coolant_temp ───────────────────> .leaf_coolantTempC [float]
├─ .ready ──────────────────────────> .leaf_invReady [bool]
├─ .fault ──────────────────────────> .leaf_invFault [bool]
├─ .warning ────────────────────────> .leaf_invWarning [bool]
├─ .limp ───────────────────────────> .leaf_invLimp [bool]
└─ .{status|temps}_update_ms ──────> .leaf_lastUpdateMs [uint32_t]
```

#### Code Locations for Field Mapping
**File:** [src/Input.cpp](src/Input.cpp#L4770-4830)

| Source Field | Target Field | Line | Freshness Check |
|---|---|---|---|
| `leafFb.rpm` | `tele.leaf_rpm` | 4780 | `lastCanRpmMonitorUpdate < 500ms` |
| `leafFbNativeStatus.torque_nm` | `tele.leaf_torqueNm` | 4793 | `leafFbNativeStatus.torque_update_ms < 500ms` |
| `leafFb.inverter_temp` | `tele.leaf_invTempC` | 4829 | `lastCanTempUpdate < 500ms` |
| `leafFb.stator_temp` | `tele.leaf_statorTempC` | 4829 | `lastCanTempUpdate < 500ms` |
| `leafFb.coolant_temp` | `tele.leaf_coolantTempC` | 4829 | `lastCanTempUpdate < 500ms` |
| `leafFb.input_voltage` | `tele.vcuHvVoltage` | 4845 | `leafFbNativeStatus.rpm_update_ms < 500ms` |
| `leafFb.{ready\|fault\|warning\|limp}` | `tele.leaf_inv{Ready\|Fault\|Warning\|Limp}` | 4835-4838 | Status freshness |

### Stage 3: WebSocket Broadcast
**File:** [src/Input.cpp](src/Input.cpp#L3024)  
**Function:** `void notifyClients(const MetaSense::Telemetry &data, bool isRecording)` (lines 3024-3150+)  
**Purpose:** Serialize telemetry struct to JSON and broadcast via WebSocket

#### Message Type 1: Dashboard Data
**Frequency:** 100ms cadence (reduced to 100ms when browser connected, 25ms otherwise)  
**Message Type:** `"data"`

**JSON Fields (relevant to CAN telemetry):**
```json
{
  "type": "data",
  "leaf_rpm": 3500.1,
  "leaf_torque": 150.25,
  "leaf_torque_demand": 155.00,
  "leaf_1da_input_v": 0,
  "leaf_1da_torque_nm": 150.25,
  "leaf_1da_rpm": 3500.1,
  "leaf_1da_clock": 0,
  "leaf_1da_err": 0,
  "leaf_1da_crc": 0,
  "leaf_1da_crc_calc": 0,
  "leaf_1da_crc_ok": 0,
  "leaf_1da_inv_status_bit": 0,
  "leaf_1da_inv_fault_map": 0,
  "leaf_1da_inv_blinky": 0,
  "leaf_1da_inv_unknown_faults": 0,
  "inv_ready": 1,
  ... (40+ more fields)
}
```

#### Message Type 2: CAN Monitor Data
**Frequency:** 250ms cadence (reduced from 100ms when browser connected)  
**Message Type:** `"canmonitor"`

**JSON Fields (CAN-focused subset):**
```json
{
  "type": "canmonitor",
  "ip": "192.168.1.100",
  "rssi": -45,
  "rpm": 3500.1,
  "leaf_rpm": 3500.1,
  "leaf_inv_temp": 65.5,
  "leaf_stator_temp": 72.3,
  "leaf_coolant_temp": 58.2,
  "leaf_1da_input_v": 0,
  "leaf_1da_torque_nm": 150.25,
  "leaf_1da_rpm": 3500.1,
  "leaf_1da_clock": 0,
  "leaf_1da_err": 0,
  "leaf_1da_crc": 0,
  ... (40+ diagnostic fields)
}
```

#### Transmission
**File:** [src/Input.cpp](src/Input.cpp#L3048) and [src/Input.cpp](src/Input.cpp#L3100)
```cpp
wsock.textAll(dataJson);      // Broadcast to all connected clients
wsock.textAll(canJson);       // CAN monitor broadcast
```

---

## Complete Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         CAN BUS (Physical Layer)                         │
│  0x1DA: MotorSpeed     0x1DC: Temps       0x55A: Alt Temps               │
│  (RPM, Torque, V)      (Inverter, etc)     (Motor, Board, etc)          │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  CANBus::poll()  [src/CANBus.cpp:880-1005]                              │
│  └─ s_canHal.receive() → raw frame (8-byte payload)                     │
│  └─ CRC validation gate for 0x1DA (only good CRC → decode)              │
│  └─ Check if ID accepted (0x1DA, 0x1DC, 0x55A)                          │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  LeafCan::decodeFrame()  [src/LeafCan.cpp:395-500+]                     │
│                                                                          │
│  0x1DA Path:                                                             │
│    ├─ Bytes 0-1: decode RPM candidates (ZE1 + legacy)                   │
│    ├─ Bytes 0-3: decode voltage, torque candidates                      │
│    ├─ Byte 7:    decode clock, error codes, CRC, status, faults         │
│    └─ selectRpmCandidate() → fb.rpm ✓                                   │
│                                                                          │
│  0x1DC Path:                                                             │
│    ├─ Extract 3 temperature values                                       │
│    └─ → fb.{inverter_temp, stator_temp, coolant_temp} ✓                 │
│                                                                          │
│  0x55A Path:                                                             │
│    ├─ Extract 4 explicit temperature channels                            │
│    └─ → fb.{inverter_temp, stator_temp, coolant_temp} ✓                 │
│                                                                          │
│  Status Path:                                                            │
│    └─ → fb.{ready, fault, warning, limp} ✓                              │
│                                                                          │
│  All paths update timestamps:                                            │
│    └─ fb.{rpm|torque|temps|status}_update_ms = now_ms                   │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  CANBus Storage  [src/CANBus.cpp:121]                                   │
│  ┌──────────────────────────────────────────────┐                       │
│  │  static LeafInvFeedback s_feedback{};        │                       │
│  │                                              │                       │
│  │  .rpm, .torque_nm, .input_voltage           │                       │
│  │  .inverter_temp, .stator_temp, .coolant_... │                       │
│  │  .ready, .fault, .warning, .limp             │                       │
│  │  ._update_ms timestamps, counters, raw data  │                       │
│  └──────────────────────────────────────────────┘                       │
│                                                                          │
│  Access via:  const LeafInvFeedback& CANBus::feedback()                  │
│               [src/CANBus.cpp:1058]                                      │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  pollLeafCanFrames()  [src/Input.cpp:2736-2810]                         │
│  └─ Fetch: const LeafInvFeedback& leafFb = CANBus::feedback()           │
│  └─ Validate: leafFb.{rpm|temps|status}_update_ms freshness < 500ms     │
│  └─ Update local caches (leafCanRpmMonitor, canInvTempC, etc)           │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  computeAndPublishTelemetry()  [src/Input.cpp:4770-4830]                │
│                                                                          │
│  Read from CANBus::feedback() AGAIN:                                     │
│  ├─ leafFb.rpm ────────────────→ tele.leaf_rpm                          │
│  ├─ leafFbNativeStatus.torque_nm → tele.leaf_torqueNm                   │
│  ├─ leafFb.{inverter|stator|coolant}_temp → tele.leaf_*TempC            │
│  ├─ leafFb.{ready|fault|warning|limp} → tele.leaf_inv*                  │
│  └─ leafFb.input_voltage ─────────→ tele.vcuHvVoltage                   │
│                                                                          │
│  Telemetry struct now contains:                                          │
│  MetaSense::Telemetry {                                                  │
│    float leaf_rpm = 3500.1;                                              │
│    float leaf_torqueNm = 150.25;                                         │
│    float leaf_invTempC = 65.5;                                           │
│    float leaf_statorTempC = 72.3;                                        │
│    float leaf_coolantTempC = 58.2;                                       │
│    bool leaf_invReady = true;                                            │
│    bool leaf_invFault = false;                                           │
│    ... (rest of dyno telemetry fields)                                   │
│  }                                                                        │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  publishTelemetry()  [src/Input.cpp:3190-3205]                          │
│  └─ Get latest: MetaSense::Telemetry = RunStorage::latest()             │
│  └─ Call: notifyClients(telemetry, isRecording)                         │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  notifyClients()  [src/Input.cpp:3024-3150+]                            │
│  ┌─────────────────────────────────────────────────────────┐            │
│  │  Message Type "data" (100ms cadence)                    │            │
│  │  {"type":"data",                                        │            │
│  │   "leaf_rpm":3500.1,                                    │            │
│  │   "leaf_torque":150.25,                                 │            │
│  │   "leaf_torque_demand":155.0,                           │            │
│  │   "inv_ready":1,                                        │            │
│  │   ... (40+ fields)}                                      │            │
│  └─────────────────────────────────────────────────────────┘            │
│  ┌─────────────────────────────────────────────────────────┐            │
│  │  Message Type "canmonitor" (250ms cadence)              │            │
│  │  {"type":"canmonitor",                                  │            │
│  │   "leaf_rpm":3500.1,                                    │            │
│  │   "leaf_inv_temp":65.5,                                 │            │
│  │   "leaf_stator_temp":72.3,                              │            │
│  │   "leaf_coolant_temp":58.2,                             │            │
│  │   ... (47+ fields)}                                      │            │
│  └─────────────────────────────────────────────────────────┘            │
│                                                                          │
│  Broadcast:  wsock.textAll(dataJson)                                     │
│              wsock.textAll(canJson)                                      │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│              WebSocket → Connected Browser Clients                       │
│           (index.html receives and displays in real-time)                │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Key Architectural Points

### 1. **Singleton Pattern**
- Global `s_feedback` struct owned by CANBus module
- All decode operations write to this single instance
- Multiple readers (Input.cpp, main.cpp, diagnostics) all access via `CANBus::feedback()`

### 2. **CRC Gating**
- **CRITICAL:** 0x1DA frames with bad CRC are NOT decoded
- File: [src/CANBus.cpp:950-955]
- Only frames passing `is1daWireCrcKnownGood()` enter decode pipeline

### 3. **Dual-Candidate Selection**
- RPM: Two decode methods (ZE1 Motorola vs Legacy LE), pick best via `selectRpmCandidate()`
- Torque: Two methods (ZE1 vs legacy), pick best (prefer ZE1 if in valid range -500 to +500 Nm)
- Allows compatibility with different Leaf model years

### 4. **Freshness Gates**
- Telemetry fields default to 0.0f if not recently updated
- Timeout: **500ms** (see `CAN_TEMP_TIMEOUT_MS`)
- Files: [src/Input.cpp:4778-4779] (RPM), [src/Input.cpp:4791-4792] (Temps)

### 5. **Diagnostic Counters**
- `fb.rpm_frames`, `fb.torque_frames`, `fb.temps_frames`, `fb.status_frames`
- Incremented per valid frame
- Accessible via WebSocket for health monitoring

### 6. **Broadcast Cadence Throttling**
- Dashboard: **100ms** normal (25ms without browser)
- CAN Monitor: **250ms** normal (100ms without browser)
- Purpose: Prevent network task starvation during OTA uploads
- File: [src/Input.cpp:3042-3043]

---

## Testing & Validation Checkpoints

| Checkpoint | Location | Validation |
|---|---|---|
| Raw frame capture | `CANBus::poll()` stats | `s_stats.rx1daFrames`, `rx1daWireCrcOk` |
| CRC validation | `CANBus::poll()` line 950 | `frame1daWireCrcOk` must be 1 to proceed |
| Decode success | `LeafCan::decodeFrame()` | Fields in `s_feedback` populated, `_update_ms` updated |
| Freshness check | `pollLeafCanFrames()` line 2778 | `(now - leafFb.rpm_update_ms) < 500ms` |
| Telemetry build | `computeAndPublishTelemetry()` | `tele.leaf_rpm`, `tele.leaf_torqueNm` populated |
| JSON broadcast | `notifyClients()` line 3048 | Both "data" and "canmonitor" messages contain fields |
| Browser display | `data/index.html` | Real-time gauge updates |

---

## Summary: Critical Dependencies

```
CAN RX Poll Cycle
  ↓ requires: TWAI driver ready
  ↓ produces: raw frame with CRC
  
CRC Validation
  ↓ requires: approved CRC8 algorithm (polynomial 0x1D, init 0xFF, xorOut 0xFF)
  ↓ produces: frame1daWireCrcOk flag
  
Decode Gate
  ↓ requires: CRC good (frame1daWireCrcOk == 1)
  ↓ produces: LeafInvFeedback struct populated
  
Telemetry Publish
  ↓ requires: LeafInvFeedback fields fresh (< 500ms old)
  ↓ produces: JSON with leaf_rpm, leaf_torqueNm, leaf_*TempC
  
WebSocket Broadcast
  ↓ requires: connected clients
  ↓ produces: browser GUI updates every 100-250ms
```

---

## Files Summary

| File | Purpose | Key Functions |
|---|---|---|
| `include/LeafCan.h` | Struct definitions | `struct LeafInvFeedback`, `LeafCan::decodeFrame()` |
| `src/LeafCan.cpp` | Frame decode logic | Decode 0x1DA/0x1DC/0x55A fields |
| `include/CANBus.h` | CANBus interface | `feedback()`, `poll()`, `send()` |
| `src/CANBus.cpp` | CAN polling & storage | Global `s_feedback`, TWAI driver integration, CRC gate |
| `include/Telemetry.h` | Telemetry schema | `struct Telemetry` with `leaf_*` fields |
| `src/Input.cpp` | Telemetry orchestration | `pollLeafCanFrames()`, `computeAndPublishTelemetry()`, `notifyClients()`, JSON formatting |
| `data/index.html` | Browser UI | WebSocket event handlers, gauge rendering |

