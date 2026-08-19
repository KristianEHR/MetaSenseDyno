# MetaSense Dyno

MetaSense dyno firmware and web UI for ESP32-S3, built with PlatformIO.

## What This Repo Contains

- Firmware: task-based control, sensor acquisition, safety interlocks, outputs.
- Web UI (LittleFS): live dashboard, settings, trend/report views.
- Persistent settings and run storage.

## Environments

- `esp32s3-USB`: local flashing over USB.
- `esp32s3-ota`: OTA flashing to device on network.

## Quick Start

Build:

```powershell
platformio run -e esp32s3-USB
```

Upload firmware:

```powershell
platformio run -e esp32s3-USB -t upload
```

Upload web UI (LittleFS):

```powershell
platformio run -e esp32s3-USB -t uploadfs
```

OTA equivalents:

```powershell
platformio run -e esp32s3-ota -t upload
platformio run -e esp32s3-ota -t uploadfs
```

## Current Runtime (Important)

- Control loop period: `25 ms` (40 Hz).
- Modbus publisher period: `50 ms`.
- WebSocket telemetry publish limiter: `50 ms`.

Task split:

- Control task: computes telemetry and physical outputs.
- Network task: WebSocket/UI/OTA.
- Modbus task: register publishing.

Physical outputs are updated by control task cadence, not by GUI refresh rate.

## Current CRC Algorithm Status

Scope:

- RX validation focus: CAN ID `0x1DA`
- TX generation focus: CAN IDs `0x1D4` and `0x11A`

Verified active method for `0x1DA`:

1. Build CRC input as `[0xDA + b0..b6]`
2. Compute CRC-8 MSB-first with:
	- poly: `0x85`
	- init: `0x00`
	- xorOut: `0xBF`
3. No extra clock residue layer is applied.
4. Final CRC:
	- `crc = crc8_msb(poly=0x85, init=0x00, xorOut=0xBF, bytes=[0xDA + b0..b6])`

Runtime policy tied to CRC quality:

- Any single BAD `0x1DA` frame is discarded for decode use (including RPM update path).
- CAN fallback trust is removed after `10` consecutive BAD frames.

Verified active method for `0x1D4`:

1. Build CRC input as `[idLo + b0..b6]`
2. Compute base CRC-8 MSB-first with:
	- poly: `0x1D`
	- init: `0xFF`
	- xorOut: `0xFF`
	- `base = crc8_msb(poly=0x1D, init=0xFF, xorOut=0xFF, bytes=[idLo + b0..b6])`
3. Derive final wire CRC by HCM clock residue (clock from byte4 bits 6..7):
	- `clock=0 -> residue 0x6C`
	- `clock=1 -> residue 0xCB`
	- `clock=2 -> residue 0xA7`
	- `clock=3 -> residue 0x00`
4. Final CRC:
	- `crc = base XOR residue[clock]`

Verified live accepted `0x1D4` baseline family:

- `b0=0xF7`
- `b1=0x07`
- `b5=0x44`
- `b6=0x30` (ChargeStatus)
- Example accepted frame: `F7 07 00 00 87 44 30 7B`

`0x11A` note:

- Runtime encoder follows the provided mux/startup layout.
- Byte 7 is startup/mux payload data for selector `m0..m3` (not a CRC byte).

Leaf VCM frame layouts used by the runtime encoder:

- `0x11A`:

```text
BO_ 282 x11A: 8 VCM
 SG_ JoystickGearPosition : 4|4@1+ (1,0) [0|0] "-" Vector__XXX
 SG_ CarOnOffStatus : 13|3@1+ (1,0) [0|0] "-" Vector__XXX
 SG_ Mulitplexor M : 48|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ StartupDataUnknown0 m0 : 56|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ StartupDataUnknown1 m1 : 56|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ StartupDataUnknown2 m2 : 56|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ StartupDataUnknown3 m3 : 56|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_: 16|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ HeartbeatVCM : 24|8@1+ (1,0) [85|170] "" Vector__XXX
 SG_ Unknown_11A_4_0 m0 : 32|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ Unknown_11A_4_1 m1 : 32|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ Unknown_11A_4_2 m2 : 32|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ Unknown_11A_4_3 m3 : 32|8@1+ (1,0) [0|0] "" Vector__XXX
 SG_ ECOselected : 12|1@1+ (1,0) [0|0] "" Vector__XXX
```

- `0x1D4`:

```text
BO_ 468 x1D4: 8 VCM
 SG_ MotorAmpTorqueRequest : 23|12@0- (0.25,0) [0|1024] "Nm" Vector__XXX
 SG_ HCM_CLOCK : 38|2@1+ (1,0) [0|3] "-" Vector__XXX
 SG_ StatusOfHighVoltagePowerSupply : 34|1@1+ (1,0) [0|0] "-" Vector__XXX
 SG_ Relay_Plus_Output_Status : 46|1@1+ (1,0) [0|0] "-" Vector__XXX
 SG_ CRC_1D4 : 56|8@1+ (1,0) [0|255] "" Vector__XXX
 SG_ ChargeStatus : 48|8@1+ (1,0) [0|255] "MODEMASK" Vector__XXX
```

Detailed report:

- `docs/CRC_ALGORITHM_REPORT_2026-08-15.md`

## Load-Cell Filtering

Two selectable modes:

- Mode A (`moving_avg`): Stage-1 moving average + alpha stage.
- Mode B (`two_stage_ma`): Stage-1 moving average + Stage-2 moving average.

Window parameters:

- `loadAvgN` = Stage-1 window.
- `loadAvgN2` = Stage-2 window (Mode B only).
- Valid range: `1..48`.

At `T_s = 0.025 s`, two-stage MA delay is approximately:

$$
	au \approx \frac{N_1 + N_2 - 2}{2} \cdot T_s
$$

Example: `N1 = N2 = 48` gives about `1.175 s` filter delay.

## Display vs Telemetry Precision

- Firmware sends high-resolution telemetry (e.g. power/torque/load with finer decimals).
- GUI rounds independently for readability (power and torque shown with 1 decimal).

This keeps charts/storage precise without making the dashboard noisy.

## Safety Notes

- VCU ready (`RB+`) is enforced as a run interlock.
- If interlock drops, runtime moves to safe output state.
- Respect hardware limits and verify wiring before high-load operation.

## Runtime HWSM State Machine

Source of truth: `src/HardwareOutputStateMachine.cpp`.

Primary states:

- `INIT`
- `IDLE`
- `MOTOR`
- `DYNO`

Transition evaluation order (every control update):

1. `INIT` if inverter status is not initialized.
2. `INIT` if precharge has failed (`prestart warning` latched).
3. `INIT` if precharge is not yet completed.
4. `INIT` if inverter is not ready.
5. `IDLE` if setpoint is at or below `500 RPM` and RPM satisfies idle hysteresis.
6. `DYNO` if setpoint is above `500 RPM` and RPM satisfies dyno hysteresis.
7. `MOTOR` otherwise.

State transition debounce:

- Candidate state must be stable for `50 ms` before commit.

RPM hysteresis currently applied:

- Idle hysteresis around `500 RPM`:
	- Enter `IDLE` at `RPM <= 400`
	- Stay `IDLE` until `RPM > 600`
- Dyno hysteresis around setpoint (only when `setpoint > 500 RPM`):
	- Enter `DYNO` at `RPM >= setpoint + 100`
	- Stay `DYNO` while `RPM >= setpoint - 100`

Precharge sub-flow (inside `INIT`):

- Minimum precharge dwell: `2500 ms`
- HV-ready threshold: `300 V`
- Current success criterion in code:
	- `HV >= 300 V`, or
	- `(elapsed >= 2500 ms) and inverterReady`
- Timeout handling at `2500 ms` if still not successful:
	- Test mode (`METASENSE_PRECHARGE_TEST_ACCEPT_TIMEOUT=1`): accept completion.
	- Normal mode (`METASENSE_PRECHARGE_TEST_ACCEPT_TIMEOUT=0`): latch prestart warning/failure and hold in `INIT` with relays off.

Relay intent by state (firmware output mapping):

- `INIT`: SSR on, RB- off, precharge active during sequence, RB+ after precharge success.
- `IDLE`: RB- on, SSR off, RB+ held (post-precharge).
- `MOTOR`: SSR on, RB- off, RB+ held.
- `DYNO`: RB- on, SSR off, RB+ held.
- Any fault or precharge-failed latch: all relays forced off.

Mermaid overview:

```mermaid
stateDiagram-v2
		[*] --> INIT
		INIT --> INIT: !statusInitialized
		INIT --> INIT: prechargeFailed
		INIT --> INIT: !prechargeCompleted
		INIT --> INIT: !inverterReady
		INIT --> IDLE: setpoint<=500 && idleHys
		INIT --> DYNO: setpoint>500 && dynoHys
		INIT --> MOTOR: else

		IDLE --> INIT: any INIT gate true
		MOTOR --> INIT: any INIT gate true
		DYNO --> INIT: any INIT gate true

		IDLE --> DYNO: setpoint>500 && dynoHys
		IDLE --> MOTOR: else

		MOTOR --> IDLE: setpoint<=500 && idleHys
		MOTOR --> DYNO: setpoint>500 && dynoHys

		DYNO --> IDLE: setpoint<=500 && idleHys
		DYNO --> MOTOR: !dynoHys

		note right of INIT
			idleHys: enter <=400, hold <=600
			dynoHys: enter >=sp+100, hold >=sp-100
			transitions debounced 50ms
		end note
```

## Useful Files

- `src/main.cpp`: task scheduling, startup, web server routes.
- `src/Input.cpp`: sensor pipeline, filters, control-side telemetry flow.
- `src/CANBus.cpp`: CAN RX/TX, wire-level 0x1DA CRC verification.
- `src/Settings.cpp`: persisted settings and validation.
- `src/CommandRouter.cpp`: WebSocket command handling and settings API.
- `include/LeafCrc.h`: global single-source CRC implementation used by runtime CRC checks/generation.
- `data/index.html`: main dashboard UI.
- `data/settings.html`: settings UI.
- `docs/NISSAN_LEAF_1DA_CRC.md`: Nissan-compatible 0x1DA CRC algorithm and residue tables.
- `docs/CRC_ALGORITHM_REPORT_2026-08-15.md`: implementation report, capture findings, and validation status.

## Typical Dev Loop

1. Edit firmware (`src/**`) and/or UI (`data/**`).
2. Build and upload firmware when C++ changed.
3. Upload LittleFS when HTML/JS/CSS changed.
4. Verify behavior on dashboard and on physical outputs.

## Git Hygiene

Before committing:

```powershell
git status --short
```

Recommended split commits:

- firmware logic
- web UI
- docs

- Adjust Kp via Pot3 in small increments while monitoring overshoot and settling.
- Tune Ki in settings only after Kp behavior is stable.
- Re-test interlock behavior after each significant tuning change.