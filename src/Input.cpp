#include "Input.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

#include "controlTask.h"
#include "DynoStateMachine.h"
#include "Settings.h"
#include "RunStorage.h"


namespace MetaSense::HardwareOutputStateMachine {

void begin();
void update(float engineThrottlePercent, float setPoint, float rpm, float primaryBrakePercent);
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

// Fast 200 Hz pre-filter states (anti-aliasing before user filter)
static float rpmPre     = 0.0f;
static float loadPre    = 0.0f;

// RPM inputs
static float canRpm   = 0.0f;
static float tachoRpm = 0.0f;
static float tachoCal = 10.0f;   // tachogen calibration factor
static float zeroOffset = 0.0f;
static float calibrationFactor = 1.0f;

// CAN RPM validity timeout + plausibility
static uint32_t lastCanRpmUpdate   = 0;
static const uint32_t CAN_RPM_TIMEOUT_MS = 100;
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

// helpers
float lpFilter(float prev, float input, float alpha)
{
    return prev + alpha * (input - prev);
}

// Frequency-correct LP filter: alpha from cutoff frequency and sample interval.
// α = 1 - exp(-2π * fc * dt)  — stable for any loop rate.
float lpFilterHz(float prev, float input, float cutoffHz, float dtSec)
{
    if (dtSec <= 0.0f) return prev;
    const float a = 1.0f - expf(-2.0f * 3.14159265f * cutoffHz * dtSec);
    return prev + a * (input - prev);
}

// ESP32-S3 ADC map provided for this hardware revision.
constexpr uint8_t kRpmSetpointPin = 1; // ADC1_CH0
constexpr uint8_t kThrottlePotPin = 2; // ADC1_CH1
constexpr uint8_t kTachoPin = 3;       // ADC1_CH2
constexpr uint8_t kDrumPin = 4;        // ADC1_CH3
constexpr uint8_t kKpAnalogPin = 5;    // ADC1_CH4

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
    return v * tachoCal;
}

float readDrumRpm()     { return readAdcSafe(kDrumPin) * 0.1f; }
float readLoadKg()      { return readAdcSafe(kThrottlePotPin) * 0.01f; }
float readEgtHotC()     { return readAdcSafe(kKpAnalogPin) * 0.25f; }
float readAmbientC()    { return 20.0f; }
float readPressureHpa() { return 1013.25f; }
float readAirDensity()  { return 1.225f; }
float readHumidity()    { return 50.0f; }
float readETorque()     { return 0.0f; }

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
        t.torqueNm      = t.loadKg * 9.82f;
        t.brakeTorqueNm = t.torqueNm;

        torqueFilt = lpFilter(torqueFilt, t.torqueNm, MetaSense::Settings::filterAlpha);
        t.torqueNm = torqueFilt;

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

    // Skip this send if any connected client has a full message queue.
    // Prevents the "Too many messages queued: closing connection" error
    // that occurs when the browser can't drain the WebSocket fast enough.
    for (auto& client : wsock.getClients()) {
        if (client.queueIsFull()) {
            return;
        }
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
    json += "\"kw\":" + String(data.kw, 2) + ",";
    json += "\"torque\":" + String(data.torqueNm, 2) + ",";
    json += "\"brakeTorque\":" + String(data.brakeTorqueNm, 2) + ",";
    json += "\"load_kg\":" + String(data.loadKg, 0) + ",";
    json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
    json += "\"peakTorque\":" + String(data.peakTorque, 2) + ",";
    json += "\"peakTorque_RPM\":" + String(data.peakTorque_RPM, 0) + ",";
    json += "\"air_density\":" + String(data.airDensity, 3) + ",";
    json += "\"ambient_temp\":" + String(data.ambientC, 1) + ",";
    json += "\"pressure\":" + String(data.pressureHpa, 1) + ",";
    json += "\"gear_ratio\":" + String(data.eTorque, 2) + ",";
    json += "\"e_torque\":" + String(data.eTorque, 2) + ",";
    json += "\"energy\":" + String(data.energyMJ, 2) + ",";
    json += "\"rel_humidity\":" + String(data.humidity, 1) + ",";
    json += "\"ratio_confidence\":" + String(data.humidity / 100.0f, 3) + ",";
    json += "\"egt_hot\":" + String(data.egtHotC, 1) + ",";
    json += "\"dyno_mode\":\"" + String(MetaSense::toString(data.mode)) + "\",";

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
    MetaSense::WebSocketServer::socket().textAll(json);
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

void tare()
{
    zeroOffset = filteredAdc;
}

void setCalibrationFactor(float factor)
{
    calibrationFactor = factor;
    MetaSense::RunStorage::saveCalibration();
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

void updateCanRpm(float rpm)
{
    if (fabs(rpm - lastCanRpm) > CAN_MAX_JUMP)
        return;

    if (rpm > 0 && rpm < 20000) {
        canRpm = rpm;
        lastCanRpm = rpm;
        lastCanRpmUpdate = millis();
    }
}

void begin()
{
    tele = MetaSense::Telemetry();
    zeroOffset = 0.0f;
    calibrationFactor = 1.0f;
    filteredAdc = 0.0f;
    MetaSense::ControlTask::configurePI(MetaSense::Settings::kp,
                                        MetaSense::Settings::ki,
                                        TORQUE_MIN,
                                        TORQUE_MAX);

    MetaSense::HardwareOutputStateMachine::begin();
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
    uint32_t now = millis();
    float dtSec = (now - last) / 1000.0f;
    last = now;

    float alpha = MetaSense::Settings::filterAlpha;

    // RPM source selectable from settings: Leaf CAN or tachogen analog.
    // In inertia mode, tachogen is mandatory.
    tachoRpm = readTachoRpm();
    bool canValid = (millis() - lastCanRpmUpdate) < CAN_RPM_TIMEOUT_MS;
    float rpmRaw = 0.0f;
    const bool forceTachoInInertia = MetaSense::Settings::inertiaMode;
    if (!forceTachoInInertia && MetaSense::Settings::useCanLeafRpm) {
        // Default to Leaf CAN RPM, but fall back to tachogen when CAN times out.
        canFallbackActive = !canValid;
        activeRpmFromCan = canValid;
        rpmRaw = canValid ? canRpm : tachoRpm;
    } else {
        canFallbackActive = false;
        activeRpmFromCan = false;
        rpmRaw = tachoRpm;
    }

    // 200 Hz pre-filter on tachogen path (CAN RPM comes pre-filtered from motor controller)
    if (!canValid || !MetaSense::Settings::useCanLeafRpm) {
        rpmPre = lpFilterHz(rpmPre, rpmRaw, 200.0f, dtSec);
        rpmRaw = rpmPre;
    }

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

    // 200 Hz pre-filter on loadcell ADC before user-configurable slow filter
    loadPre = lpFilterHz(loadPre, loadRaw, 200.0f, dtSec);
    loadRaw = loadPre;

    drumRpmFilt = lpFilter(drumRpmFilt, drumRaw, alpha);
    loadKgFilt  = lpFilter(loadKgFilt, loadRaw, alpha);
    filteredAdc = loadKgFilt;

    tele.drumRpm = drumRpmFilt;
    tele.loadKg  = (filteredAdc - zeroOffset) * calibrationFactor;

    tele.egtHotC     = readEgtHotC();
    tele.egtAmbientC = readAmbientC();
    tele.ambientC    = readAmbientC();
    tele.pressureHpa = readPressureHpa();
    tele.airDensity  = readAirDensity();
    tele.humidity    = readHumidity();

    const float manualRpmTarget = readRpmSetpointPot();
    MetaSense::DynoStateMachine::setManualRpmTarget(manualRpmTarget);
    MetaSense::DynoStateMachine::update();

    if (MetaSense::DynoStateMachine::isAutoRunActive()) {
        // In autorun, preserve target provided by state machine path.
        tele.rpmTarget = MetaSense::Settings::getRpmTarget();
    } else {
        // In manual mode, pot is the active RPM target source.
        tele.rpmTarget = manualRpmTarget;
        MetaSense::Settings::setRpmTarget(tele.rpmTarget);
    }

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

    const float primaryBrakeSignedPercent = (torqueCmd / TORQUE_MAX) * 100.0f;
    MetaSense::HardwareOutputStateMachine::update(
        readThrottlePotPercent(),
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

    static uint32_t lastSend = 0;
    if (now - lastSend >= 50) {
        lastSend = now;
        notifyClients(tele, tele.recording);
    }
}

} // namespace MetaSense::Input
