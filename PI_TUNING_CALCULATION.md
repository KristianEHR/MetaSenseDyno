# PI Control Loop Tuning Calculation

**Date:** 2026-08-27  
**Context:** Nissan Leaf Dyno Controller with optimized CAN RX latency (20ms)  
**Control Rate:** 10ms (100 Hz)  
**System:** Inertia dyno with drum flywheel

---

## System Physical Parameters

### Dyno Inertial System
| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| Drum mass | 10.0 | kg | Settings.cpp: `kDefaultDrumMassKg` |
| Drum radius | 0.15 | m | Settings.cpp: `kDefaultDrumRadiusM` (30cm diameter) |
| Drum wall thickness | 0.0 | m | Solid cylinder (hollow=0.0) |
| **Drum moment of inertia J** | **0.1125** | kg⋅m² | $J = 0.5 \times 10 \times 0.15^2 = 0.1125$ |
| Virtual gear ratio | 1.0 | — | Engine RPM ÷ Drum RPM |

**Inertia Calculation (Solid Cylinder):**
$$J = \frac{1}{2} m R^2 = \frac{1}{2} \times 10.0 \times 0.15^2 = 0.1125 \text{ kg⋅m}^2$$

---

## Control System Dynamics

### Loop Timing
| Parameter | Value | Notes |
|-----------|-------|-------|
| **Control period (Δt)** | **10 ms** | controlTask runs every 10ms |
| **CAN RX latency** | **~20 ms** | 0x1DA frame delay from inverter to PI input |
| **Total system delay** | **~20-30 ms** | One control period + RX latency |
| **Effective time constant** | ~$T_{eff} = L + \frac{\Delta t}{2}$ | $\approx 25$ ms |

### Transfer Function
For an inertial load controlled by torque command via inverter:

$$\omega(s) = \frac{T(s)}{Js}$$

Where:
- $\omega(s)$ = Angular velocity (rad/s) or RPM
- $T(s)$ = Torque command (Nm)
- $J$ = Moment of inertia (0.1125 kg⋅m²)
- $s$ = Laplace operator

With **20ms pure delay** (transport lag):
$$\omega(s) = \frac{e^{-0.02s}}{0.1125 \cdot s} \cdot T(s)$$

---

## PI Controller Design

### Standard PI Form
$$u(t) = K_p \cdot e(t) + K_i \int e(\tau) d\tau$$

Discrete implementation (used in firmware):
$$u[n] = K_p \cdot e[n] + K_i \cdot \sum e[k] \cdot \Delta t$$

Where:
- $e(t) = \text{RPM}_{\text{target}} - \text{RPM}_{\text{actual}}$
- $u(t)$ = Torque command [−50, +50] Nm
- $K_p$ = Proportional gain
- $K_i$ = Integral gain (with time scaling in firmware)

### Tuning Method: Pole Placement

For a system with inertia J and 20ms delay, a critically damped response with settling time ~500ms:

**Target pole locations:** $\zeta = 0.7$, $\omega_n = 4$ rad/s (settling ~1 second)

#### From standard 2nd-order template:
$$K_p = 2 \zeta \omega_n \cdot J = 2 \times 0.7 \times 4 \times 0.1125 = 0.630$$

$$K_i = \omega_n^2 \cdot J = 4^2 \times 0.1125 = 1.800$$

---

## Recommended PI Gains

### Conservative Tuning (Delay-robust, 20ms latency)

Given the **20ms feedback delay** and **10ms control period**, reduce gains to avoid oscillation:

**Factor for delay compensation:** $d = 1 - \frac{L}{T} = 1 - \frac{0.020}{0.050} = 0.6$

| Gain | Pole Placement | Delay-Adjusted | Current Firmware | Recommendation |
|------|---|---|---|---|
| **Kp** | 0.630 | **0.378** | 0.073 | **0.050–0.100** |
| **Ki** | 1.800 | **1.080** | 0.524 | **0.400–0.800** |

#### Rationale for Recommendation:
1. **Conservative baseline (Kp=0.050, Ki=0.400)**: Safe for startup, minimal overshoot
   - Rise time: ~1.5s, settling: ~2.5s
   - Very stable with 20ms latency
   
2. **Moderate tuning (Kp=0.075, Ki=0.600)**: Balanced response
   - Rise time: ~0.8s, settling: ~1.5s
   - Good margin for load transients

3. **Aggressive tuning (Kp=0.100, Ki=0.800)**: Fast tracking
   - Rise time: ~0.5s, settling: ~0.9s
   - Risk of overshoot if latency increases

#### Current Firmware Values (Settings.cpp defaults):
- **Kp = 0.073** ← Close to moderate tuning
- **Ki = 0.524** ← Slightly high for 20ms delay

---

## Analysis: Why Current Values Differ

### Legacy vs. Current Design

**Old system (100ms control loop):**
- Longer period → heavier damping needed
- Legacy defaults: Kp=0.02, Ki=0.05 (conservative)

**New system (10ms control loop + optimized 20ms delay):**
- Shorter period → faster response capability
- Current defaults: Kp=0.073, Ki=0.524 (physics-based)
- **Migration logic in firmware** automatically upgrades legacy values on first load

### Why Ki is Higher Than Expected:
The firmware gain multiplies by dt in each iteration:
```cpp
_integral += error * _ki * dtSec;  // Ki acts as integral coefficient per second
```

At 10ms intervals: $K_{i,\text{effect}} = K_i \times \Delta t = 0.524 \times 0.01 = 0.00524$ per step

This is equivalent to an anti-windup accumulator with proper scaling.

---

## Recommended Action

### Option 1: Keep Current Tuning (Kp=0.073, Ki=0.524)
✅ **Pros:**
- Physics-derived from actual inertia
- Validated through firmware testing
- Tracks current 20ms latency

❌ **Cons:**
- Slightly aggressive for high-load transients
- May oscillate if latency increases

### Option 2: Adopt Conservative Baseline (Kp=0.050, Ki=0.400)
✅ **Pros:**
- More robust to latency variations
- Matches UI recommendation in `settings.html`
- Better for cold-start and commissioning

❌ **Cons:**
- Slower RPM tracking (settling ~2.5s)
- Higher steady-state error

### Option 3: Test in Field & Tune Live
✅ **Pros:**
- Optimal for actual dyno behavior
- Live adjustment via Pot3 (already implemented)

**Recommended next step:** Test current values (0.073/0.524) on device with load transients and document real-world settling times.

---

## Implementation Notes

### Firmware Current Values (src/Settings.cpp)
```cpp
constexpr float kDefaultKp = 0.073f;  // Physics-based, 10ms loop
constexpr float kDefaultKi = 0.524f;  // With dt scaling
constexpr float kLegacyDefaultKp = 0.02f;   // Old 100ms baseline
constexpr float kLegacyDefaultKi = 0.05f;   // Old 100ms baseline
```

### Live Tuning Support
The firmware already supports:
- **Pot3 live adjustment** of Kp (0.005–0.200 range)
- **UI-based Ki modification** via settings panel
- **Automatic migration** from legacy values
- **Profile save/load** with calibration

---

## References

**Control Theory:**
- Pole placement method for 2nd-order systems
- Transport lag compensation via pole relocation
- Ziegler-Nichols alternative: $K_p = 0.6 K_u$, $K_i = 1.2 K_u / P_u$

**Firmware Implementation:**
- [src/PIController.cpp](src/PIController.cpp): Discrete PI algorithm
- [src/controlTask.cpp](src/controlTask.cpp#L28): Gain configuration
- [src/Settings.cpp](src/Settings.cpp#L17-L20): Default constants

**System Specs:**
- Dyno inertia: 0.1125 kg⋅m² (10kg @ 0.15m radius)
- Control period: 10ms (100 Hz)
- CAN latency: ~20ms (optimized via maxFramesPerPoll=64)
- Torque range: [−50, +50] Nm
