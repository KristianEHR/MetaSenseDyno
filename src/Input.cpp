#include "Input.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <math.h>
#include <Wire.h>

#include <Adafruit_BME680.h>
#include <Adafruit_NAU7802.h>

#include "controlTask.h"
#include "DynoStateMachine.h"
#include "Settings.h"
#include "RunStorage.h"
#include "TempHAL.h"
#include "globals.h"


namespace MetaSense::HardwareOutputStateMachine {

void begin();
void update(float engineThrottlePercent, float setPoint, float rpm, float primaryBrakePercent);
bool isMotorState();
void stop();

} // namespace MetaSense::HardwareOutputStateMachine

namespace MetaSense::WebSocketServer {

AsyncWebSocket& socket();

} // namespace MetaSense::WebSocketServer


namespace { // LOCAL SCOPE

static MetaSense::Telemetry tele;

// filter state
static float rpmFilt     = 0.0f;
static float drumRpmFilt = 0.0f;
static float loadKgFilt  = 0.0f;
static float torqueFilt  = 0.0f;
static float filteredAdc = 0.0f;
constexpr uint8_t kLoadRawAverageWindow = 10;
static float loadRawAverageBuffer[kLoadRawAverageWindow] = {0.0f};
static uint8_t loadRawAverageCount = 0;
static uint8_t loadRawAverageIndex = 0;
static float loadRawAverageSum = 0.0f;
constexpr uint8_t kTachoRawAverageWindow = kLoadRawAverageWindow;
static float tachoRawAverageBuffer[kTachoRawAverageWindow] = {0.0f};
static uint8_t tachoRawAverageCount = 0;
static uint8_t tachoRawAverageIndex = 0;
static float tachoRawAverageSum = 0.0f;

// RPM inputs
static float canRpm   = 0.0f;
static float tachoRpm = 0.0f;
static float tachoCal = 10.0f;   // tachogen calibration factor
static float zeroOffset = 0.0f;
static float zeroDeadbandRaw = 0.0f;
static float calibrationFactor = 0.01f;
static float torqueResidualOffsetNm = 0.0f;
static TempHAL egtDigital;
static bool egtDigitalReady = false;
static String i2cScanSummary = "";
static Adafruit_BME680 ambientBme;
static Adafruit_NAU7802 loadCellNau;
static bool ambientBmeReady = false;
static bool ambientBmeReadPending = false;
static uint32_t ambientBmeReadReadyMs = 0;
static bool loadCellNauReady = false;
static uint32_t lastLoadCellNauRetryMs = 0;
static float ambientTempC = 20.0f;
static float ambientHumidityPct = 50.0f;
static float ambientPressureHpa = 1013.25f;
static uint32_t lastAmbientSampleMs = 0;

// CAN RPM validity timeout + plausibility
static uint32_t lastCanRpmUpdate   = 0;
static const uint32_t CAN_RPM_TIMEOUT_MS = 100;
static const uint32_t CAN_RPM_MIN_UPDATE_MS = 20;
static float lastCanRpm            = 0.0f;
static const float CAN_MAX_JUMP    = 2000.0f;

// RPM delta error
static bool  rpmDeltaError         = false;
static bool  canFallbackActive     = false;
static bool  activeRpmFromCan      = false;
static const float RPM_DELTA_LIMIT = 100.0f;

// safety limits
constexpr float RPM_MAX_LIMIT   = 18000.0f;
constexpr float EGT_MAX_LIMIT_C = 950.0f;

// PI controller
constexpr float TORQUE_MIN = -200.0f;
constexpr float TORQUE_MAX =  200.0f;
constexpr float RPM_SETPOINT_MAX = RPM_MAX_LIMIT;
constexpr uint32_t kWebSocketPublishPeriodMs = 50;
constexpr float kRuntimeKpMin = 0.005f;
constexpr float kRuntimeKpMax = 0.200f;
constexpr float kRuntimeKpAlpha = 0.12f;
constexpr float kRuntimeKpApplyDelta = 0.001f;
constexpr uint32_t kAmbientSamplePeriodMs = 1000;
constexpr uint32_t kLoadCellNauRetryPeriodMs = 5000;
constexpr bool kEgtOnlyI2cInitMode = false;
constexpr bool kNauOnlyI2cInitMode = false;

// helpers
float lpFilter(float prev, float input, float alpha)
{
    return prev + alpha * (input - prev);
}

void resetLoadRawAverage(float seed)
{
    loadRawAverageCount = 0;
    loadRawAverageIndex = 0;
    loadRawAverageSum = 0.0f;

    for (uint8_t i = 0; i < kLoadRawAverageWindow; ++i) {
        loadRawAverageBuffer[i] = 0.0f;
    }

    if (isfinite(seed)) {
        loadRawAverageBuffer[0] = seed;
        loadRawAverageSum = seed;
        loadRawAverageCount = 1;
        loadRawAverageIndex = 1;
    }
}

float applyLoadRawAverage(float sample)
{
    if (!isfinite(sample)) {
        return sample;
    }

    if (loadRawAverageCount < kLoadRawAverageWindow) {
        loadRawAverageBuffer[loadRawAverageIndex] = sample;
        loadRawAverageSum += sample;
        ++loadRawAverageCount;
    } else {
        loadRawAverageSum -= loadRawAverageBuffer[loadRawAverageIndex];
        loadRawAverageBuffer[loadRawAverageIndex] = sample;
        loadRawAverageSum += sample;
    }

    ++loadRawAverageIndex;
    if (loadRawAverageIndex >= kLoadRawAverageWindow) {
        loadRawAverageIndex = 0;
    }

    return loadRawAverageSum / static_cast<float>(loadRawAverageCount);
}

void resetTachoRawAverage(float seed)
{
    tachoRawAverageCount = 0;
    tachoRawAverageIndex = 0;
    tachoRawAverageSum = 0.0f;

    for (uint8_t i = 0; i < kTachoRawAverageWindow; ++i) {
        tachoRawAverageBuffer[i] = 0.0f;
    }

    if (isfinite(seed)) {
        tachoRawAverageBuffer[0] = seed;
        tachoRawAverageSum = seed;
        tachoRawAverageCount = 1;
        tachoRawAverageIndex = 1;
    }
}

float applyTachoRawAverage(float sample)
{
    if (!isfinite(sample)) {
        return sample;
    }

    if (tachoRawAverageCount < kTachoRawAverageWindow) {
        tachoRawAverageBuffer[tachoRawAverageIndex] = sample;
        tachoRawAverageSum += sample;
        ++tachoRawAverageCount;
    } else {
        tachoRawAverageSum -= tachoRawAverageBuffer[tachoRawAverageIndex];
        tachoRawAverageBuffer[tachoRawAverageIndex] = sample;
        tachoRawAverageSum += sample;
    }

    ++tachoRawAverageIndex;
    if (tachoRawAverageIndex >= kTachoRawAverageWindow) {
        tachoRawAverageIndex = 0;
    }

    return tachoRawAverageSum / static_cast<float>(tachoRawAverageCount);
}

// ESP32-S3 ADC map provided for this hardware revision.
constexpr uint8_t kRpmSetpointPin = 1; // ADC1_CH0
constexpr uint8_t kThrottlePotPin = 2; // ADC1_CH1
constexpr uint8_t kTachoPin = 3;       // ADC1_CH2
constexpr uint8_t kKpPotPin = 6;       // ADC1_CH5 (pot3 – runtime Kp, moved from GPIO4 to free CAN TX)
constexpr uint8_t kLoadCellPin = 32;   // Load-cell analog input
// ADC1_CH4 (GPIO 5) is available for future use
// Drum RPM is derived from tachogen × (1/virtGearRatio)

static float kpPotFilteredAdc = -1.0f;
static float lastAppliedKp = -1.0f;

// Digital inputs
static bool prevSwState = false;
static uint32_t swDebounceMs = 0;
constexpr uint32_t kSwDebounceThresholdMs = 30;

float readAdcSafe(uint8_t pin)
{
    if (digitalPinToAnalogChannel(pin) < 0) {
        static uint32_t lastWarnMs = 0;
        const uint32_t now = millis();
        if (now - lastWarnMs > 3000) {
            lastWarnMs = now;
            Serial.printf("[Input] Invalid ADC pin on this board: %u\n", pin);
        }
        return 0.0f;
    }
    return static_cast<float>(analogRead(pin));
}

float readTachoRpm()
{
    float v = readAdcSafe(kTachoPin);
    v = applyTachoRawAverage(v);
    return v * tachoCal;
}

float readDrumRpm()
{
    // Drum RPM is derived from the tachogen by inverting the configured gear ratio.
    // virtGearRatio = engineRpm / drumRpm, so drumRpm = tachoRpm / virtGearRatio.
    const float ratio = MetaSense::Settings::virtGearRatio;
    return (ratio > 0.01f) ? (tachoRpm / ratio) : tachoRpm;
}
float readLoadKg()
{
    if (loadCellNauReady) {
        if (loadCellNau.available()) {
            // NAU7802 returns raw signed ADC counts; scaling is applied later
            // via zero offset + calibration factor in the telemetry path.
            return static_cast<float>(loadCellNau.read());
        }
        // Keep returning last filtered RAW value if no fresh conversion is ready.
        return filteredAdc;
    }

    // Backward-compatible fallback for boards wired without NAU7802.
    return readAdcSafe(kLoadCellPin) * 0.01f;
}

bool sampleLoadRawStats(float& outAverageRaw, float& outMaxAbsDeviation, uint16_t timeoutMs = 220, uint8_t maxSamples = 24)
{
    float samples[24];
    uint8_t count = 0;
    const uint32_t start = millis();

    while ((millis() - start) < timeoutMs && count < maxSamples) {
        const float raw = readLoadKg();
        if (isfinite(raw)) {
            samples[count++] = raw;
        }
        delay(5);
    }

    if (count == 0) {
        return false;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < count; ++i) {
        sum += samples[i];
    }
    outAverageRaw = sum / static_cast<float>(count);

    float maxDev = 0.0f;
    for (uint8_t i = 0; i < count; ++i) {
        const float dev = fabsf(samples[i] - outAverageRaw);
        if (dev > maxDev) {
            maxDev = dev;
        }
    }
    outMaxAbsDeviation = maxDev;

    return true;
}

void tryInitLoadCellNau()
{
    if (loadCellNauReady) {
        return;
    }

    loadCellNauReady = loadCellNau.begin(&Wire);
    if (loadCellNauReady) {
        loadCellNau.setGain(NAU7802_GAIN_128);
        loadCellNau.setRate(NAU7802_RATE_80SPS);
        Serial.println("[Input] Load-cell ADC source ready (NAU7802 @ 0x2A)");
        Serial0.println("[Input] Load-cell ADC source ready (NAU7802 @ 0x2A)");
    } else {
        Serial.println("[Input] Load-cell ADC source unavailable (NAU7802), using GPIO32 ADC fallback");
        Serial0.println("[Input] Load-cell ADC source unavailable (NAU7802), using GPIO32 ADC fallback");
    }
}

void updateAmbientInputs(bool forceSample)
{
    if (!ambientBmeReady) {
        return;
    }

    const uint32_t now = millis();

    // Complete any in-flight forced conversion without blocking the loop.
    if (ambientBmeReadPending) {
        const bool readyToFinish = forceSample ||
            (static_cast<int32_t>(now - ambientBmeReadReadyMs) >= 0);
        if (readyToFinish) {
            if (ambientBme.endReading()) {
                const float bmeTemp = ambientBme.temperature;
                const float bmeHumidity = ambientBme.humidity;
                const float pressurePa = ambientBme.pressure;

                if (isfinite(bmeTemp) && bmeTemp > -50.0f && bmeTemp < 120.0f) {
                    ambientTempC = bmeTemp;
                }
                if (isfinite(bmeHumidity)) {
                    float humidityWithOffset = bmeHumidity + MetaSense::Settings::ambientRhOffsetPct;
                    if (humidityWithOffset < 0.0f) humidityWithOffset = 0.0f;
                    if (humidityWithOffset > 100.0f) humidityWithOffset = 100.0f;
                    ambientHumidityPct = humidityWithOffset;
                }
                if (isfinite(pressurePa) && pressurePa > 10000.0f && pressurePa < 120000.0f) {
                    ambientPressureHpa = pressurePa / 100.0f;
                }
            }
            ambientBmeReadPending = false;
            lastAmbientSampleMs = now;
        }
    }

    if (ambientBmeReadPending) {
        return;
    }

    if (!forceSample && (now - lastAmbientSampleMs) < kAmbientSamplePeriodMs) {
        return;
    }

    const uint32_t readyAtMs = ambientBme.beginReading();
    if (readyAtMs != 0) {
        ambientBmeReadPending = true;
        ambientBmeReadReadyMs = readyAtMs;
    }
}

float readEgtHotC()
{
    if (!egtDigital.isReady()) {
        return 0.0f;
    }

    const float egt = egtDigital.readHotC();
    if (!isfinite(egt) || egt < 0.0f || egt > 1800.0f) {
        return 0.0f;
    }

    return egt;
}

float readEgtAmbientC()
{
    if (!egtDigital.isReady()) {
        return 0.0f;
    }

    const float ambient = egtDigital.readAmbientC();
    if (!isfinite(ambient) || ambient < -50.0f || ambient > 200.0f) {
        return 0.0f;
    }

    return ambient;
}

float readAmbientC()    { updateAmbientInputs(false); return ambientTempC; }
float readPressureHpa() { updateAmbientInputs(false); return ambientPressureHpa; }
float computeAirDensityKgM3(float tempC, float pressureHpa, float humidityPct)
{
    // Magnus formula for saturation vapour pressure (hPa)
    const float Psat = 6.1078f * expf(17.269f * tempC / (237.3f + tempC));
    const float Pv   = (humidityPct / 100.0f) * Psat;  // partial vapour pressure, hPa
    const float Pd   = pressureHpa - Pv;               // dry-air partial pressure, hPa
    const float T_K  = tempC + 273.15f;
    // Ideal gas: ρ = Pd/(Rd*T) + Pv/(Rv*T)
    const float Pd_Pa = Pd * 100.0f;
    const float Pv_Pa = Pv * 100.0f;
    const float rho = (Pd_Pa / (287.058f * T_K)) + (Pv_Pa / (461.495f * T_K));
    return (rho > 0.0f) ? rho : 1.225f;
}

float readAirDensity()  { return computeAirDensityKgM3(ambientTempC, ambientPressureHpa, ambientHumidityPct); }
float readHumidity()    { updateAmbientInputs(false); return ambientHumidityPct; }
float readETorque()     { return 0.0f; }

void updateI2cScanSummary()
{
    String summary;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            if (!summary.isEmpty()) {
                summary += ",";
            }
            char buf[6];
            snprintf(buf, sizeof(buf), "0x%02X", addr);
            summary += buf;
        }
    }
    i2cScanSummary = summary.isEmpty() ? String("none") : summary;
}

void applyRuntimeKpFromPot(bool forceApply)
{
    const float adc = readAdcSafe(kKpPotPin);
    if (kpPotFilteredAdc < 0.0f) {
        kpPotFilteredAdc = adc;
    } else {
        kpPotFilteredAdc = lpFilter(kpPotFilteredAdc, adc, kRuntimeKpAlpha);
    }

    float normalized = kpPotFilteredAdc / 4095.0f;
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    const float kpFromPot = kRuntimeKpMin + normalized * (kRuntimeKpMax - kRuntimeKpMin);
    if (!forceApply && fabsf(kpFromPot - lastAppliedKp) < kRuntimeKpApplyDelta) {
        return;
    }

    MetaSense::ControlTask::configurePI(kpFromPot,
                                        MetaSense::Settings::ki,
                                        TORQUE_MIN,
                                        TORQUE_MAX);
    lastAppliedKp = kpFromPot;
}

float readRpmSetpointPot()
{
    const float adc = readAdcSafe(kRpmSetpointPin);
    float rpmSetpoint = (adc / 4095.0f) * RPM_SETPOINT_MAX;
    if (rpmSetpoint < 0.0f) rpmSetpoint = 0.0f;
    if (rpmSetpoint > RPM_SETPOINT_MAX) rpmSetpoint = RPM_SETPOINT_MAX;
    return rpmSetpoint;
}

float readThrottlePotPercent()
{
    const float adc = readAdcSafe(kThrottlePotPin);
    float percent = (adc / 4095.0f) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

// Persistent angular velocity for inertia differentiation.
static float omegaPrev = 0.0f;

void updateDyno(MetaSense::Telemetry& t, float dtSec)
{
    if (MetaSense::Settings::inertiaMode) {
        // --- Inertia dyno path ---
        // ω_drum (rad/s)
        const float omega = t.drumRpm * (2.0f * 3.14159265f / 60.0f);

        // Angular acceleration α = dω/dt, lightly filtered
        float alpha = 0.0f;
        if (dtSec > 0.001f) {
            alpha = lpFilter(0.0f, (omega - omegaPrev) / dtSec,
                             MetaSense::Settings::filterAlpha);
        }
        omegaPrev = omega;

        // T_drum = J · α
        const float J = MetaSense::Settings::drumInertiaKgM2;
        const float torqueDrum = J * alpha;
        torqueFilt = lpFilter(torqueFilt, torqueDrum, MetaSense::Settings::filterAlpha);

        // Scale to engine side: T_engine = T_drum × ratio
        const float ratio = MetaSense::Settings::virtGearRatio;
        t.torqueNm      = torqueFilt * ratio;
        t.brakeTorqueNm = t.torqueNm;

        // Power is invariant: P = T_drum · ω_drum  (watts → kW)
        const float powerW = torqueFilt * omega;
        t.kw = powerW / 1000.0f;

        t.energyMJ += (powerW * dtSec) / 1000000.0f;

    } else {
        // --- Brake / load-cell dyno path (original) ---
        const float armCm = (MetaSense::Settings::armCm > 0.01f) ? MetaSense::Settings::armCm : 0.01f;
        const float rawTorqueNm = (t.loadKg * 9.82f / 100.0f) * armCm;
        t.torqueNm      = rawTorqueNm - torqueResidualOffsetNm;
        t.brakeTorqueNm = t.torqueNm;

        // --- Climate correction: normalise torque/power to standard conditions ---
        // Reference: 1013.25 hPa, 20 °C, 0 % RH  (ρ_ref ≈ 1.2041 kg/m³)
        const float rhoActual = computeAirDensityKgM3(ambientTempC, ambientPressureHpa, ambientHumidityPct);
        const float rhoRef    = computeAirDensityKgM3(20.0f, 1013.25f, 0.0f);
        t.climateCF = (rhoActual > 0.3f) ? (rhoRef / rhoActual) : 1.0f;
        t.torqueNm      *= t.climateCF;
        t.brakeTorqueNm  = t.torqueNm;

        // P(W) = T(Nm) * ω(rad/s) = T * rpm * 2π / 60
        // Correct for drivetrain losses to get crank power.
        const float eff = MetaSense::Settings::drivetrainEff / 100.0f;
        const float omegaEngine = t.rpm * (2.0f * 3.14159265f / 60.0f);
        const float powerW = (eff > 0.01f) ? (t.torqueNm * omegaEngine / eff) : (t.torqueNm * omegaEngine);
        t.kw = powerW / 1000.0f;
        t.energyMJ += (powerW * dtSec) / 1000000.0f;

        omegaPrev = t.drumRpm * (2.0f * 3.14159265f / 60.0f); // keep in sync
    }
}

bool checkSafety(const MetaSense::Telemetry& t)
{
    if (t.rpm > RPM_MAX_LIMIT)       return false;
    if (t.egtHotC > EGT_MAX_LIMIT_C) return false;
    return true;
}

void notifyClients(const MetaSense::Telemetry &data, bool isRecording)
{
    AsyncWebSocket& wsock = MetaSense::WebSocketServer::socket();
    if (wsock.count() == 0) {
        return;
    }

    String json;
    json.reserve(512);

    json = "{";
    json += "\"type\":\"data\",";
    json += "\"rpm\":" + String(data.rpm, 0) + ",";
    json += "\"rpm_error\":" + String(rpmDeltaError ? 1 : 0) + ",";
    json += "\"can_fallback\":" + String(canFallbackActive ? 1 : 0) + ",";
    json += "\"rpm_source_active\":\"" + String(activeRpmFromCan ? "leafrpm" : "tachogen") + "\",";
    json += "\"drum_rpm\":" + String(data.drumRpm, 0) + ",";
    json += "\"rpm_target\":" + String(data.rpmTarget, 0) + ",";
    json += "\"kp_source\":\"" + String(MetaSense::Settings::usePot3Kp ? "pot3" : "firmware") + "\",";
    json += "\"kp_live\":" + String(MetaSense::Settings::usePot3Kp && lastAppliedKp >= 0.0f ? lastAppliedKp : MetaSense::Settings::kp, 4) + ",";
    json += "\"ki_live\":" + String(MetaSense::Settings::ki, 4) + ",";
    json += "\"throttle_pct\":" + String(data.throttlePercent, 0) + ",";
    json += "\"kw\":" + String(data.kw, 2) + ",";
    json += "\"torque\":" + String(data.torqueNm, 2) + ",";
    json += "\"brakeTorque\":" + String(data.brakeTorqueNm, 2) + ",";
    json += "\"load_kg\":" + String(data.loadKg, 1) + ",";
    json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
    json += "\"peakTorque\":" + String(data.peakTorque, 2) + ",";
    json += "\"peakTorque_RPM\":" + String(data.peakTorque_RPM, 0) + ",";
    json += "\"air_density\":" + String(data.airDensity, 3) + ",";
    json += "\"climate_cf\":" + String(data.climateCF, 4) + ",";
    json += "\"ambient_temp\":" + String(data.ambientC, 1) + ",";
    json += "\"pressure\":" + String(data.pressureHpa, 1) + ",";
    json += "\"gear_ratio\":" + String(data.eTorque, 2) + ",";
    json += "\"e_torque\":" + String(data.eTorque, 2) + ",";
    json += "\"energy\":" + String(data.energyMJ, 2) + ",";
    json += "\"rel_humidity\":" + String(data.humidity, 1) + ",";
    json += "\"ratio_confidence\":" + String(data.humidity / 100.0f, 3) + ",";
    json += "\"egt_hot\":" + String(data.egtHotC, 1) + ",";
    json += "\"egt_status\":" + String(data.egtStatus) + ",";
    json += "\"egt_ready\":" + String(data.egtReady ? 1 : 0) + ",";
    json += "\"egt_addr\":" + String(data.egtAddress) + ",";
    json += "\"egt_ack_addr\":" + String(data.egtAckAddress) + ",";
    json += "\"i2c_scan\":\"" + i2cScanSummary + "\",";
    json += "\"load_source\":\"" + String(loadCellNauReady ? "nau7802" : "adc32") + "\",";
    json += "\"nau_ready\":" + String(loadCellNauReady ? 1 : 0) + ",";
    json += "\"nau_data_ready\":" + String((loadCellNauReady && loadCellNau.available()) ? 1 : 0) + ",";
    json += "\"heartbeat_ms\":" + String(millis()) + ",";
    json += "\"dyno_mode\":\"" + String(MetaSense::toString(data.mode)) + "\",";
    json += "\"vcu_ready\":" + String(data.vcuReady ? 1 : 0) + ",";
    json += "\"sw_active\":" + String(data.swActive ? 1 : 0) + ",";

    json += "\"mcps\":{";
    float hot = data.egtHotC;
    float amb = data.egtAmbientC;

    if (hot > 0 && hot < 1500) {
        json += "\"7\":{\"hot\":" + String(hot, 1) +
                ",\"amb\":" + String(amb, 1) + "}";
    } else {
        json += "\"7\":{\"hot\":0,\"amb\":0}";
    }

    json += "}";
    json += "}";
    for (auto& client : wsock.getClients()) {
        if (!client.canSend() || client.queueIsFull()) {
            continue;
        }
        client.text(json);
    }
}

void publishTelemetry()
{
    static uint32_t lastPublishedVersion = 0;
    static uint32_t lastSendMs = 0;

    const uint32_t now = millis();
    if (now - lastSendMs < kWebSocketPublishPeriodMs) {
        return;
    }

    const uint32_t currentVersion = MetaSense::RunStorage::version();
    if (currentVersion == lastPublishedVersion) {
        return;
    }

    lastPublishedVersion = currentVersion;
    lastSendMs = now;
    const MetaSense::Telemetry telemetry = MetaSense::RunStorage::latest();
    notifyClients(telemetry, MetaSense::DynoStateMachine::isRecording());
}

void notifyRunComplete(const MetaSense::Telemetry& data)
{
    if (MetaSense::WebSocketServer::socket().count() == 0) {
        return;
    }

    String json;
    json.reserve(192);
    json = "{\"type\":\"run_complete\"";
    json += ",\"peakKW\":" + String(data.kw, 2);
    json += ",\"peakKW_RPM\":" + String(data.maxRpm, 0);
    json += ",\"peakTorque\":" + String(data.peakTorque, 2);
    json += ",\"peakTorque_RPM\":" + String(data.peakTorque_RPM, 0);
    json += ",\"peakEGT\":" + String(data.egtHotC, 1);
    json += "}";
    MetaSense::WebSocketServer::socket().textAll(json);
}

} // anonymous namespace


namespace MetaSense::Input { // EXTERNAL SCOPE

void tareMainGui()
{
    tare();

    float avgRaw = 0.0f;
    float maxDev = 0.0f;
    if (sampleLoadRawStats(avgRaw, maxDev)) {
        float netRaw = avgRaw - zeroOffset;
        if (fabsf(netRaw) < zeroDeadbandRaw) {
            netRaw = 0.0f;
        }

        const float residualLoadKg = netRaw * calibrationFactor;
        const float armCm = (MetaSense::Settings::armCm > 0.01f) ? MetaSense::Settings::armCm : 0.01f;
        torqueResidualOffsetNm = (residualLoadKg * 9.82f / 100.0f) * armCm;
    } else {
        torqueResidualOffsetNm = 0.0f;
    }
}

void tare()
{
    float avgRaw = 0.0f;
    float maxDev = 0.0f;
    if (sampleLoadRawStats(avgRaw, maxDev)) {
        zeroOffset = avgRaw;
        zeroDeadbandRaw = max(2.0f, maxDev * 3.0f);
        filteredAdc = avgRaw;
        resetLoadRawAverage(avgRaw);
        loadKgFilt = 0.0f;
    } else {
        zeroOffset = filteredAdc;
        zeroDeadbandRaw = 2.0f;
        resetLoadRawAverage(filteredAdc);
        loadKgFilt = 0.0f;
    }

    // Force immediate zero output on the load-cell torque path after tare.
    torqueFilt = 0.0f;
    tele.loadKg = 0.0f;
    tele.torqueNm = 0.0f;
    tele.brakeTorqueNm = 0.0f;
}

void setCalibrationFactor(float factor)
{
    calibrationFactor = factor;
    MetaSense::RunStorage::saveCalibration();
}

bool calibrateWithKnownWeight(float knownWeightKg, float& outFactor)
{
    if (!(knownWeightKg > 0.0f) || !isfinite(knownWeightKg)) {
        return false;
    }

    float avgRaw = filteredAdc;
    float maxDev = 0.0f;
    (void)sampleLoadRawStats(avgRaw, maxDev);

    // Calibration uses current averaged raw sensor counts relative to tare offset.
    const float netRaw = avgRaw - zeroOffset;
    if (!isfinite(netRaw) || fabsf(netRaw) < 1e-6f) {
        return false;
    }

    const float factor = knownWeightKg / netRaw;
    if (!isfinite(factor)) {
        return false;
    }

    setCalibrationFactor(factor);
    outFactor = factor;
    return true;
}

float getCalibrationFactor()
{
    return calibrationFactor;
}

float getZeroOffset()
{
    return zeroOffset;
}

float rpm()
{
    return tele.rpm;
}

float torqueNm()
{
    return tele.torqueNm;
}

bool isVcuReady()
{
    return tele.vcuReady;
}

void updateCanRpm(float rpm)
{
    const uint32_t now = millis();
    if (now - lastCanRpmUpdate < CAN_RPM_MIN_UPDATE_MS) {
        return;
    }

    if (fabs(rpm - lastCanRpm) > CAN_MAX_JUMP)
        return;

    if (rpm > 0 && rpm < 20000) {
        canRpm = rpm;
        lastCanRpm = rpm;
        lastCanRpmUpdate = now;
    }
}

void begin()
{
    tele = MetaSense::Telemetry();
    torqueResidualOffsetNm = 0.0f;
    zeroOffset = 0.0f;
    calibrationFactor = 0.01f;
    filteredAdc = 0.0f;
    resetLoadRawAverage(filteredAdc);
    resetTachoRawAverage(0.0f);
    {
        float storedZero = 0.0f;
        float storedFactor = 0.01f;
        if (MetaSense::RunStorage::loadCalibration(storedZero, storedFactor)) {
            if (isfinite(storedZero)) {
                zeroOffset = storedZero;
                filteredAdc = storedZero;
                resetLoadRawAverage(filteredAdc);
            }
            if (isfinite(storedFactor) && storedFactor > 0.0f) {
                calibrationFactor = storedFactor;
            }
        }
    }
    kpPotFilteredAdc = -1.0f;
    lastAppliedKp = -1.0f;
    MetaSense::ControlTask::configurePI(MetaSense::Settings::kp,
                                        MetaSense::Settings::ki,
                                        TORQUE_MIN,
                                        TORQUE_MAX);
    if (MetaSense::Settings::usePot3Kp) {
        applyRuntimeKpFromPot(true);
    }

    // Digital inputs
    pinMode(MetaSense::Globals::kRampSwitchPin, INPUT_PULLUP);  // SW switch – active LOW
    pinMode(MetaSense::Globals::kRbPlusInputPin, INPUT_PULLUP); // VCU ready – active LOW
    prevSwState = (digitalRead(MetaSense::Globals::kRampSwitchPin) == LOW);

    // Bring relay/control outputs to a known-safe state before probing I2C sensors.
    MetaSense::HardwareOutputStateMachine::begin();

    if (kEgtOnlyI2cInitMode || kNauOnlyI2cInitMode) {
        ambientBmeReady = false;
        ambientBmeReadPending = false;
        Serial.println("[Input] NAU-only I2C mode: skipping BME680 initialization");
        Serial0.println("[Input] NAU-only I2C mode: skipping BME680 initialization");
    } else {
        ambientBmeReady = ambientBme.begin(0x76) || ambientBme.begin(0x77);
        ambientBmeReadPending = false;
        if (ambientBmeReady) {
            ambientBme.setTemperatureOversampling(BME680_OS_1X);
            ambientBme.setHumidityOversampling(BME680_OS_1X);
            ambientBme.setPressureOversampling(BME680_OS_1X);
            ambientBme.setIIRFilterSize(BME680_FILTER_SIZE_0);
            ambientBme.setGasHeater(0, 0);
            Serial.println("[Input] Ambient source ready (BME680)");
            Serial0.println("[Input] Ambient source ready (BME680)");
        } else {
            Serial.println("[Input] Ambient source unavailable (BME680)");
            Serial0.println("[Input] Ambient source unavailable (BME680)");
        }
    }

    if (!kNauOnlyI2cInitMode) {
        egtDigitalReady = egtDigital.begin();
        if (egtDigitalReady) {
            Serial.println("[Input] EGT digital source ready (MCP9600)");
            Serial0.println("[Input] EGT digital source ready (MCP9600)");
        } else {
            Serial.println("[Input] EGT digital source unavailable");
            Serial0.println("[Input] EGT digital source unavailable");
        }
    } else {
        egtDigitalReady = false;
        Serial.println("[Input] NAU-only I2C mode: skipping MCP9600 initialization");
        Serial0.println("[Input] NAU-only I2C mode: skipping MCP9600 initialization");
    }

    tryInitLoadCellNau();

    updateI2cScanSummary();
    Serial.printf("[Input] I2C devices: %s\n", i2cScanSummary.c_str());
    Serial0.printf("[Input] I2C devices: %s\n", i2cScanSummary.c_str());

    updateAmbientInputs(true);
}

void startRecording()
{
    tele.recording = true;

    tele.peakTorque     = 0.0f;
    tele.peakTorque_RPM = 0.0f;

    tele.maxRpm         = 0.0f;
    tele.maxTorqueNm    = 0.0f;

    MetaSense::ControlTask::resetPI();
}

void stopRecording()
{
    tele.recording = false;
    MetaSense::HardwareOutputStateMachine::stop();
}

void loop()
{
    static bool prevRecording = false;
    static uint32_t last = millis();
    static uint32_t lastEgtRetryMs = 0;
    uint32_t now = millis();
    float dtSec = (now - last) / 1000.0f;
    last = now;

    if (!loadCellNauReady && (now - lastLoadCellNauRetryMs) >= kLoadCellNauRetryPeriodMs) {
        lastLoadCellNauRetryMs = now;
        tryInitLoadCellNau();
        updateI2cScanSummary();
    }

    if (!egtDigital.isReady() && (now - lastEgtRetryMs) >= 10000) {
        lastEgtRetryMs = now;
        egtDigitalReady = egtDigital.begin();
    }

    if (MetaSense::Settings::usePot3Kp) {
        applyRuntimeKpFromPot(false);
    }

    // --- VCU ready (RB+, GPIO 36, active LOW) ---
    tele.vcuReady = true;
    if (!tele.vcuReady) {
        // Global VCU interlock: dyno cannot run unless VCU/RB+ ready is asserted.
        if (MetaSense::DynoStateMachine::isAutoRunActive()) {
            MetaSense::DynoStateMachine::setAutoMode(false);
            MetaSense::DynoStateMachine::setPanelAuto(false);
            MetaSense::DynoStateMachine::abortAutoRun();
        }
        if (MetaSense::DynoStateMachine::isRecording()) {
            MetaSense::DynoStateMachine::stopRecording();
        }
        MetaSense::Settings::setRpmTarget(0.0f);
    }

    // --- SW switch recording toggle (GPIO 35, active LOW, debounced) ---
    {
        const bool swNow = (digitalRead(MetaSense::Globals::kRampSwitchPin) == LOW);
        if (swNow != prevSwState) {
            if ((now - swDebounceMs) >= kSwDebounceThresholdMs) {
                swDebounceMs = now;
                prevSwState = swNow;
                tele.swActive = swNow;
                if (swNow && tele.vcuReady) {
                    // Rising edge + VCU ready: toggle recording
                    if (!MetaSense::DynoStateMachine::isRecording()) {
                        MetaSense::DynoStateMachine::startRecording();
                    } else {
                        MetaSense::DynoStateMachine::stopRecording();
                    }
                }
            }
        } else {
            swDebounceMs = now;
            tele.swActive = swNow;
        }
    }

    float alpha = MetaSense::Settings::filterAlpha;

    // RPM source: Leaf CAN is primary, tachogen on GPIO 3 is the active fallback.
    tachoRpm = readTachoRpm();
    bool canValid = (millis() - lastCanRpmUpdate) < CAN_RPM_TIMEOUT_MS;
    float rpmRaw = 0.0f;
    canFallbackActive = !canValid;
    activeRpmFromCan = canValid;
    rpmRaw = canValid ? canRpm : tachoRpm;

    rpmFilt = lpFilter(rpmFilt, rpmRaw, alpha);
    tele.rpm = rpmFilt;

    if (canValid) {
        float delta = fabs(canRpm - tachoRpm);
        rpmDeltaError = (delta > RPM_DELTA_LIMIT);
    } else {
        rpmDeltaError = false;
    }

    // other sensors
    float drumRaw = readDrumRpm();
    float loadRaw = readLoadKg();
    loadRaw = applyLoadRawAverage(loadRaw);

    drumRpmFilt = lpFilter(drumRpmFilt, drumRaw, alpha);
    filteredAdc = loadRaw;

    tele.drumRpm = drumRpmFilt;
    float netRaw = filteredAdc - zeroOffset;
    if (fabsf(netRaw) <= zeroDeadbandRaw) {
        netRaw = 0.0f;
    }
    const float loadKgRaw = netRaw * calibrationFactor;
    loadKgFilt = lpFilter(loadKgFilt, loadKgRaw, alpha);
    tele.loadKg  = loadKgFilt;

    updateAmbientInputs(false);
    tele.egtReady    = egtDigital.isReady();
    tele.egtStatus   = egtDigital.status();
    tele.egtAddress  = egtDigital.address();
    tele.egtAckAddress = egtDigital.ackAddress();
    tele.egtHotC     = readEgtHotC();
    tele.egtAmbientC = readEgtAmbientC();
    tele.ambientC    = readAmbientC();
    tele.pressureHpa = readPressureHpa();
    tele.airDensity  = readAirDensity();
    tele.humidity    = readHumidity();

    const float manualRpmTarget = readRpmSetpointPot();
    MetaSense::DynoStateMachine::setManualRpmTarget(manualRpmTarget);
    MetaSense::DynoStateMachine::update();

    if (!tele.vcuReady) {
        tele.rpmTarget = 0.0f;
        MetaSense::Settings::setRpmTarget(0.0f);
    } else if (MetaSense::DynoStateMachine::isAutoRunActive()) {
        // In autorun, preserve target provided by state machine path.
        tele.rpmTarget = MetaSense::Settings::getRpmTarget();
    } else {
        // In manual mode, pot is the active RPM target source.
        tele.rpmTarget = manualRpmTarget;
    }

    const float motorModeCapRpm = (MetaSense::Settings::motorModeMaxRpm > 0.0f)
        ? MetaSense::Settings::motorModeMaxRpm
        : RPM_SETPOINT_MAX;
    if (MetaSense::HardwareOutputStateMachine::isMotorState() && tele.rpmTarget > motorModeCapRpm) {
        tele.rpmTarget = motorModeCapRpm;
    }
    MetaSense::Settings::setRpmTarget(tele.rpmTarget);

    updateDyno(tele, dtSec);

    // Estimate engine-side torque from dyno torque using configured ratio.
    const float ratio = (MetaSense::Settings::virtGearRatio > 0.0f) ? MetaSense::Settings::virtGearRatio : 1.0f;
    tele.eTorque = tele.torqueNm * ratio;

    bool safe = checkSafety(tele);
    if (!safe) {
        tele.torqueNm      = 0.0f;
        tele.brakeTorqueNm = 0.0f;
    }

    float torqueCmd = MetaSense::ControlTask::computeTorqueCommand(
        tele.rpmTarget,
        tele.rpm,
        dtSec);
    if (!safe) torqueCmd = 0.0f;

    const float maxAllowedRpm = (MetaSense::Settings::maxRPM > 0.0f) ? MetaSense::Settings::maxRPM : RPM_MAX_LIMIT;
    const float maxAllowedTorque = (MetaSense::Settings::maxTorque > 0.0f) ? MetaSense::Settings::maxTorque : TORQUE_MAX;
    const bool rpmLimitExceeded = tele.rpm > maxAllowedRpm;
    const bool torqueLimitExceeded = fabsf(tele.torqueNm) > maxAllowedTorque;
    const bool throttleSafetyCut = rpmLimitExceeded || torqueLimitExceeded;

    const float primaryBrakeSignedPercent = tele.vcuReady ? (torqueCmd / TORQUE_MAX) * 100.0f : 0.0f;
    // POT2 on AD1 (GPIO2, 0-3.3V) is the engine throttle setpoint source.
    // Map directly to GPIO45 PWM as 0-100% servo output.
    const float engineThrottlePercent = throttleSafetyCut ? 0.0f : readThrottlePotPercent();
    tele.throttlePercent = engineThrottlePercent;

    MetaSense::HardwareOutputStateMachine::update(
        engineThrottlePercent,
        tele.rpmTarget,
        tele.rpm,
        primaryBrakeSignedPercent);

    if (tele.torqueNm > tele.peakTorque) {
        tele.peakTorque     = tele.torqueNm;
        tele.peakTorque_RPM = tele.rpm;
    }
    if (tele.rpm > tele.maxRpm)          tele.maxRpm      = tele.rpm;
    if (tele.torqueNm > tele.maxTorqueNm) tele.maxTorqueNm = tele.torqueNm;

    MetaSense::RunStorage::save(tele);

    if (prevRecording && !tele.recording) {
        notifyRunComplete(tele);
    }
    prevRecording = tele.recording;
}

void publish()
{
    publishTelemetry();
}

} // namespace MetaSense::Input
