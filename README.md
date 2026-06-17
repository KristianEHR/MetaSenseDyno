# MetaSense Dyno

PlatformIO-based firmware and web UI for the MetaSense dyno controller.

## Build

```powershell
platformio run -e esp32s3-USB
```

## Upload firmware

```powershell
platformio run -e esp32s3-USB -t upload
```

## Upload web files

```powershell
platformio run -e esp32s3-USB -t uploadfs
```

## OTA upload

```powershell
platformio run -e esp32s3-ota -t upload
platformio run -e esp32s3-ota -t uploadfs
```

## Notes

- Use `esp32s3-USB` for local USB flashing.
- Use `esp32s3-ota` for network uploads to `dyno-controller.local`.
- The web UI is served from LittleFS.

## Current Firmware Runtime

- ESP32-S3 FreeRTOS split:
	- Control task on Core 1 (high priority).
	- Network/WebSocket/OTA task on Core 0 (medium priority).
	- Modbus publisher task on Core 0 (low priority).
- Control loop period is 100 ms.
- WebSocket telemetry publish target is 20 ms (version-gated to skip stale payloads).
- Modbus register publish period is 50 ms.

## Safety and Interlocks

- VCU ready (RB+) is a global run interlock:
	- Run start commands are blocked when VCU ready is not asserted.
	- Runtime immediately forces safe state when VCU ready drops:
		- stop recording,
		- abort auto mode,
		- force RPM target to 0,
		- force throttle/brake command outputs to 0.
- SW switch input is debounced and toggles recording on active edge when VCU ready is true.

## Sensor and I/O Mapping

### Analog inputs (ADC1)

- GPIO 1 / ADC1_CH0: RPM setpoint pot
- GPIO 2 / ADC1_CH1: Throttle pot
- GPIO 3 / ADC1_CH2: Tachogen analog input
- GPIO 4 / ADC1_CH3: Pot3 for runtime PI Kp tuning
- GPIO 5 / ADC1_CH4: free

### Digital inputs

- GPIO 35: SW switch (recording toggle)
- GPIO 36: VCU ready / RB+

### I2C sensors (SDA=GPIO 17, SCL=GPIO 18)

- MCP9600: EGT hot junction and EGT ambient (cold-junction)
- AHT20: ambient air temperature and relative humidity
- BMP280: ambient pressure

### Outputs

- PWM (20 kHz, 8-bit):
	- GPIO 45: engine throttle
	- GPIO 40: brake channel
	- GPIO 41: dyno throttle / VCU
- Digital:
	- GPIO 38: RB- FET
	- GPIO 39: SSSR
	- GPIO 48: onboard NeoPixel status LED

## Control Notes

- Drum RPM is derived from tachogen RPM and configured ratio:
	- `drumRpm = tachoRpm / virtGearRatio` (with guard when ratio is near zero).
- PI Kp can be tuned live from Pot3; Ki remains from settings.

## Bring-Up Checklist

### 1) Power-on and wiring checks

- Verify VCU ready / RB+ input is wired and readable before attempting any run.
- Verify SW switch input state changes in telemetry (`sw_active`).
- Verify Pot3 (Kp) moves runtime Kp as expected.
- Verify tachogen input is stable at zero RPM with machine stopped.
- Verify PWM outputs are physically disconnected from actuator load for first logic-only check.

### 2) Sensor readiness checks

- Confirm MCP9600 is detected on boot (`EGT digital source ready`).
- Confirm ambient sensors report plausible values:
	- AHT20 temperature/humidity,
	- BMP280 pressure.
- Confirm EGT hot and EGT ambient fields update and are plausible at room conditions.

### 2.1) Expected telemetry ranges at idle (engine off)

Use this as a quick sanity check after boot and sensor warm-up.

| Telemetry field | Typical idle/off range | Notes |
|---|---|---|
| `rpm` | 0 to 50 | Depends on tachogen noise floor and filtering |
| `drum_rpm` | 0 to 20 | Derived from tachogen and ratio; should be near zero when stationary |
| `load_kg` | Near 0 after tare | May show offset if not tared/calibrated |
| `egt_hot` | Ambient to ~80 C | Should not be high with cold engine/exhaust |
| `egtAmbientC` (cold junction) | Ambient +/- 10 C | From MCP9600 internal ambient/cold-junction reading |
| `ambient_temp` | Site ambient (typically 10 to 40 C) | From AHT20/BMP source path |
| `rel_humidity` | 10 to 90 %RH | Environment-dependent |
| `pressure` | 950 to 1050 hPa | Approximate sea-level weather range |
| `vcu_ready` | 0 or 1 (as wired state) | Must be asserted for run start |
| `sw_active` | 0 or 1 (switch state) | Verify switch transitions are visible |

### 3) Interlock checks (must pass)

- With VCU ready OFF:
	- run start commands are blocked,
	- RPM target forced to 0,
	- throttle and brake command outputs forced to 0.
- Toggle VCU ready ON and verify start commands are accepted.
- Toggle VCU ready OFF during active run and verify immediate safe shutdown behavior.

### 4) First motion test (low risk)

- Start with conservative PI settings (current defaults for 100 ms loop).
- Use a low RPM target and verify smooth, bounded response.
- Check for oscillation, hunting, or abrupt torque transitions.
- Validate that STOP and SW switch toggling return outputs to safe state.

### 5) First dyno run validation

- Confirm drum RPM tracks expected `tachoRpm / virtGearRatio` relationship.
- Confirm torque/power trends are monotonic and plausible.
- Confirm WebSocket update responsiveness and Modbus register refresh behavior.
- Check thermal and safety limits (EGT, RPM max) are enforced.

### 6) Post-run tuning pass

- Adjust Kp via Pot3 in small increments while monitoring overshoot and settling.
- Tune Ki in settings only after Kp behavior is stable.
- Re-test interlock behavior after each significant tuning change.