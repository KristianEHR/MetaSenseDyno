#include "Settings.h"
#include <math.h>
#include <Preferences.h>

namespace {

constexpr float kDefaultRpmTarget = 0.0f;
constexpr float kDefaultTachoCal = 10.0f;
constexpr float kDefaultRpmStart = 1500.0f;
constexpr float kDefaultRpmEnd = 5500.0f;

constexpr float kDefaultFilterAlpha = 0.2f;
constexpr float kDefaultLoadAvgN = 5.0f;
constexpr float kDefaultLoadAvgN2 = 5.0f;
constexpr uint8_t kDefaultLoadFilterMode = 1;
constexpr uint16_t kDefaultLoadCellGain = 128;
constexpr uint16_t kDefaultLoadCellRateSps = 320;
constexpr float kDefaultKp = 0.073f;
constexpr float kDefaultKi = 0.524f;
constexpr float kLegacyDefaultKp = 0.02f;
constexpr float kLegacyDefaultKi = 0.05f;
constexpr bool  kDefaultUsePot3Kp = false;
constexpr float kDefaultAmbientRhOffsetPct = 0.0f;
constexpr float kDefaultMotorModeMaxRpm = 2000.0f;
constexpr float kDefaultIdleTorqueNm = 4.0f;
constexpr float kDefaultBrakeMaxTorqueNm = 30.0f;
constexpr bool kDefaultForceVcuReadyForUiTest = false;
#ifndef METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS
#define METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS 0
#endif
constexpr bool kDefaultLeafSimFeedbackEnabled = (METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS != 0);

constexpr float kDefaultMaxRPM = 18000.0f;
constexpr float kDefaultMaxHP = 25.0f;
constexpr float kDefaultMaxTorque = 200.0f;
constexpr float kDefaultArmCm = 20.0f;

constexpr float kDefaultPulsesPerRev = 1.0f;
constexpr float kDefaultPulsesPerRevDrum = 1.0f;

constexpr float kDefaultDrivetrainEff = 95.0f;
constexpr bool  kDefaultInertiaMode = false;
constexpr bool  kDefaultUseCanLeafRpm = true;
constexpr float kDefaultDrumMassKg = 10.0f;
constexpr float kDefaultDrumRadiusM = 0.15f;
constexpr float kDefaultDrumWallM = 0.0f;
constexpr float kDefaultVirtGearRatio = 1.0f;
constexpr const char* kSettingsNs = "settings";
constexpr const char* kSettingsInitKey = "init";

float rpmTargetLocal = kDefaultRpmTarget;
float tachoCalLocal = kDefaultTachoCal;
float rpmStartLocal = kDefaultRpmStart;
float rpmEndLocal = kDefaultRpmEnd;

} // anonymous namespace


namespace MetaSense::Settings {

float filterAlpha = kDefaultFilterAlpha;
float loadAvgN = kDefaultLoadAvgN;
float loadAvgN2 = kDefaultLoadAvgN2;
uint8_t loadFilterMode = kDefaultLoadFilterMode;
uint16_t loadCellGain = kDefaultLoadCellGain;
uint16_t loadCellRateSps = kDefaultLoadCellRateSps;
// Conservative baseline for 100 ms control loop cadence.
float kp = kDefaultKp;
float ki = kDefaultKi;
bool usePot3Kp = kDefaultUsePot3Kp;
float ambientRhOffsetPct = kDefaultAmbientRhOffsetPct;
float motorModeMaxRpm = kDefaultMotorModeMaxRpm;
float idleTorqueNm = kDefaultIdleTorqueNm;
float brakeMaxTorqueNm = kDefaultBrakeMaxTorqueNm;
bool forceVcuReadyForUiTest = kDefaultForceVcuReadyForUiTest;
bool leafSimFeedbackEnabled = kDefaultLeafSimFeedbackEnabled;

// Gauge display ranges
float maxRPM          = kDefaultMaxRPM;
float maxHP           = kDefaultMaxHP;
float maxTorque       = kDefaultMaxTorque;
float armCm           = kDefaultArmCm;

// Pulse inputs
float pulsesPerRev     = kDefaultPulsesPerRev;
float pulsesPerRevDrum = kDefaultPulsesPerRevDrum;

// Drivetrain efficiency
float drivetrainEff    = kDefaultDrivetrainEff;

// Inertia dyno parameters (defaults)
bool  inertiaMode     = kDefaultInertiaMode;
bool  useCanLeafRpm   = kDefaultUseCanLeafRpm;
float drumMassKg      = kDefaultDrumMassKg;   // kg
float drumRadiusM     = kDefaultDrumRadiusM;  // 30 cm diameter → 0.15 m radius
float drumWallM       = kDefaultDrumWallM;    // 0 = solid cylinder
float drumInertiaKgM2 = 0.1125f;              // recomputed by resetToDefaults
float virtGearRatio   = kDefaultVirtGearRatio; // drum→engine ratio

// Recomputes J from mass, outer radius and wall thickness.
// Solid cylinder:  J = 0.5 * m * R²
// Hollow cylinder: J = 0.5 * m * (R_outer² + R_inner²)
static void recomputeInertia()
{
    const float Ro = drumRadiusM;
    const float Ri = (drumWallM > 0.0f) ? (Ro - drumWallM) : 0.0f;
    if (Ri <= 0.0f) {
        drumInertiaKgM2 = 0.5f * drumMassKg * Ro * Ro;          // solid
    } else {
        drumInertiaKgM2 = 0.5f * drumMassKg * (Ro * Ro + Ri * Ri); // hollow
    }
}

void setDrumParams(float massKg, float radiusM, float wallM)
{
    drumMassKg  = massKg;
    drumRadiusM = radiusM;
    drumWallM   = wallM;
    recomputeInertia();
}

void setDrumInertiaCustom(float J)
{
    drumInertiaKgM2 = J;
}

void setInertiaMode(bool enabled)
{
    inertiaMode = enabled;
}

void resetToDefaults()
{
    filterAlpha = kDefaultFilterAlpha;
    loadAvgN = kDefaultLoadAvgN;
    loadAvgN2 = kDefaultLoadAvgN2;
    loadFilterMode = kDefaultLoadFilterMode;
    loadCellGain = kDefaultLoadCellGain;
    loadCellRateSps = kDefaultLoadCellRateSps;
    kp = kDefaultKp;
    ki = kDefaultKi;
    usePot3Kp = kDefaultUsePot3Kp;
    ambientRhOffsetPct = kDefaultAmbientRhOffsetPct;
    motorModeMaxRpm = kDefaultMotorModeMaxRpm;
    idleTorqueNm = kDefaultIdleTorqueNm;
    brakeMaxTorqueNm = kDefaultBrakeMaxTorqueNm;
    forceVcuReadyForUiTest = kDefaultForceVcuReadyForUiTest;
    leafSimFeedbackEnabled = kDefaultLeafSimFeedbackEnabled;

    maxRPM = kDefaultMaxRPM;
    maxHP = kDefaultMaxHP;
    maxTorque = kDefaultMaxTorque;
    armCm = kDefaultArmCm;

    pulsesPerRev = kDefaultPulsesPerRev;
    pulsesPerRevDrum = kDefaultPulsesPerRevDrum;

    drivetrainEff = kDefaultDrivetrainEff;

    inertiaMode = kDefaultInertiaMode;
    useCanLeafRpm = kDefaultUseCanLeafRpm;
    drumMassKg = kDefaultDrumMassKg;
    drumRadiusM = kDefaultDrumRadiusM;
    drumWallM = kDefaultDrumWallM;
    virtGearRatio = kDefaultVirtGearRatio;
    recomputeInertia();

    rpmTargetLocal = kDefaultRpmTarget;
    tachoCalLocal = kDefaultTachoCal;
    rpmStartLocal = kDefaultRpmStart;
    rpmEndLocal = kDefaultRpmEnd;
}

void loadFromStorage()
{
    Preferences prefs;
    if (!prefs.begin(kSettingsNs, true)) {
        return;
    }

    const bool hasData = prefs.getBool(kSettingsInitKey, false);
    if (!hasData) {
        prefs.end();
        return;
    }

    filterAlpha = prefs.getFloat("filterAlpha", filterAlpha);
    loadAvgN = prefs.getFloat("loadAvgN", loadAvgN);
    loadAvgN2 = prefs.getFloat("loadAvgN2", loadAvgN2);
    loadFilterMode = prefs.getUChar("lcFMode", loadFilterMode);
    loadCellGain = static_cast<uint16_t>(prefs.getUShort("lcGain", loadCellGain));
    loadCellRateSps = static_cast<uint16_t>(prefs.getUShort("lcRate", loadCellRateSps));
    kp = prefs.getFloat("kp", kp);
    ki = prefs.getFloat("ki", ki);
    usePot3Kp = prefs.getBool("pot3kp", usePot3Kp);
    ambientRhOffsetPct = prefs.getFloat("rhOffPct", ambientRhOffsetPct);
    motorModeMaxRpm = prefs.getFloat("motorMaxRpm", motorModeMaxRpm);
    idleTorqueNm = prefs.getFloat("idleTorque", idleTorqueNm);
    brakeMaxTorqueNm = prefs.getFloat("brakeMaxTq", brakeMaxTorqueNm);
    forceVcuReadyForUiTest = kDefaultForceVcuReadyForUiTest;
    leafSimFeedbackEnabled = prefs.getBool("leafSimFb", leafSimFeedbackEnabled);
    if (kDefaultLeafSimFeedbackEnabled) {
        leafSimFeedbackEnabled = true;
    } else {
        leafSimFeedbackEnabled = false;
    }

    // One-time migration: only bump unchanged legacy defaults to the
    // physics-based RPM-domain PI gains. Preserve user-tuned values.
    if (fabsf(kp - kLegacyDefaultKp) < 0.0005f &&
        fabsf(ki - kLegacyDefaultKi) < 0.0005f) {
        kp = kDefaultKp;
        ki = kDefaultKi;
    }

    maxRPM = prefs.getFloat("maxRPM", maxRPM);
    maxHP = prefs.getFloat("maxHP", maxHP);
    maxTorque = prefs.getFloat("maxTorque", maxTorque);
    armCm = prefs.getFloat("armCm", armCm);

    pulsesPerRev = prefs.getFloat("pprEng", pulsesPerRev);
    pulsesPerRevDrum = prefs.getFloat("pprDrum", pulsesPerRevDrum);
    drivetrainEff = prefs.getFloat("driveEff", drivetrainEff);

    inertiaMode = prefs.getBool("inertia", inertiaMode);
    useCanLeafRpm = true;
    drumMassKg = prefs.getFloat("drumMass", drumMassKg);
    drumRadiusM = prefs.getFloat("drumRad", drumRadiusM);
    drumWallM = prefs.getFloat("drumWall", drumWallM);
    virtGearRatio = prefs.getFloat("gearRatio", virtGearRatio);

    rpmTargetLocal = prefs.getFloat("rpmTarget", rpmTargetLocal);
    tachoCalLocal = prefs.getFloat("tachoCal", tachoCalLocal);
    rpmStartLocal = prefs.getFloat("rpmStart", rpmStartLocal);
    rpmEndLocal = prefs.getFloat("rpmEnd", rpmEndLocal);

    prefs.end();

    if (drumRadiusM <= 0.0f) {
        drumRadiusM = kDefaultDrumRadiusM;
    }
    if (drumMassKg <= 0.0f) {
        drumMassKg = kDefaultDrumMassKg;
    }
    if (pulsesPerRev <= 0.0f) {
        pulsesPerRev = kDefaultPulsesPerRev;
    }
    if (pulsesPerRevDrum <= 0.0f) {
        pulsesPerRevDrum = kDefaultPulsesPerRevDrum;
    }
    if (virtGearRatio <= 0.0f) {
        virtGearRatio = kDefaultVirtGearRatio;
    }
    // Backward-compatibility: some older builds stored lever arm in meters.
    // The current setting/UI is centimeters, so migrate small meter-like values.
    if (armCm > 0.0f && armCm <= 2.0f) {
        armCm *= 100.0f;
    }
    if (armCm <= 0.0f) {
        armCm = kDefaultArmCm;
    } else if (armCm > 200.0f) {
        armCm = 200.0f;
    }
    if (loadAvgN < 1.0f) {
        loadAvgN = 1.0f;
    } else if (loadAvgN > 255.0f) {
        loadAvgN = 255.0f;
    }
    if (loadAvgN2 < 1.0f) {
        loadAvgN2 = 1.0f;
    } else if (loadAvgN2 > 255.0f) {
        loadAvgN2 = 255.0f;
    }
    if (loadFilterMode > 1) {
        loadFilterMode = kDefaultLoadFilterMode;
    }
    if (!(loadCellGain == 1 || loadCellGain == 2 || loadCellGain == 4 || loadCellGain == 8 ||
          loadCellGain == 16 || loadCellGain == 32 || loadCellGain == 64 || loadCellGain == 128)) {
        loadCellGain = kDefaultLoadCellGain;
    }
    if (!(loadCellRateSps == 10 || loadCellRateSps == 20 || loadCellRateSps == 40 ||
          loadCellRateSps == 80 || loadCellRateSps == 320)) {
        loadCellRateSps = kDefaultLoadCellRateSps;
    }
    if (idleTorqueNm < 0.0f) {
        idleTorqueNm = 0.0f;
    } else if (idleTorqueNm > 10.0f) {
        idleTorqueNm = 10.0f;
    }
    if (brakeMaxTorqueNm < 0.0f) {
        brakeMaxTorqueNm = 0.0f;
    } else if (brakeMaxTorqueNm > 200.0f) {
        brakeMaxTorqueNm = 200.0f;
    }

    recomputeInertia();
}

void saveToStorage()
{
    Preferences prefs;
    if (!prefs.begin(kSettingsNs, false)) {
        return;
    }

    prefs.putBool(kSettingsInitKey, true);

    prefs.putFloat("filterAlpha", filterAlpha);
    prefs.putFloat("loadAvgN", loadAvgN);
    prefs.putFloat("loadAvgN2", loadAvgN2);
    prefs.putUChar("lcFMode", loadFilterMode);
    prefs.putUShort("lcGain", loadCellGain);
    prefs.putUShort("lcRate", loadCellRateSps);
    prefs.putFloat("kp", kp);
    prefs.putFloat("ki", ki);
    prefs.putBool("pot3kp", usePot3Kp);
    prefs.putFloat("rhOffPct", ambientRhOffsetPct);
    prefs.putFloat("motorMaxRpm", motorModeMaxRpm);
    prefs.putFloat("idleTorque", idleTorqueNm);
    prefs.putFloat("brakeMaxTq", brakeMaxTorqueNm);
    prefs.putBool("leafSimFb", leafSimFeedbackEnabled);

    prefs.putFloat("maxRPM", maxRPM);
    prefs.putFloat("maxHP", maxHP);
    prefs.putFloat("maxTorque", maxTorque);
    prefs.putFloat("armCm", armCm);

    prefs.putFloat("pprEng", pulsesPerRev);
    prefs.putFloat("pprDrum", pulsesPerRevDrum);
    prefs.putFloat("driveEff", drivetrainEff);

    prefs.putBool("inertia", inertiaMode);
    prefs.putBool("canRpm", true);
    prefs.putFloat("drumMass", drumMassKg);
    prefs.putFloat("drumRad", drumRadiusM);
    prefs.putFloat("drumWall", drumWallM);
    prefs.putFloat("gearRatio", virtGearRatio);

    prefs.putFloat("rpmTarget", rpmTargetLocal);
    prefs.putFloat("tachoCal", tachoCalLocal);
    prefs.putFloat("rpmStart", rpmStartLocal);
    prefs.putFloat("rpmEnd", rpmEndLocal);

    prefs.end();
}

void setRpmTarget(float rpm)
{
    rpmTargetLocal = rpm;
}

float getRpmTarget()
{
    return rpmTargetLocal;
}

void setRpmStart(float rpm)
{
    rpmStartLocal = rpm;
}

void setTachoCal(float cal)
{
    tachoCalLocal = cal;
}

float getTachoCal()
{
    return tachoCalLocal;
}

float rpmStart()
{
    return rpmStartLocal;
}

void setRpmEnd(float rpm)
{
    rpmEndLocal = rpm;
}

float rpmEnd()
{
    return rpmEndLocal;
}

} // namespace MetaSense::Settings
