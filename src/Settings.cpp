#include "Settings.h"
#include <math.h>
#include <Preferences.h>

namespace {

constexpr float kDefaultRpmTarget = 0.0f;
constexpr float kDefaultTachoCal = 10.0f;
constexpr float kDefaultRpmStart = 1500.0f;
constexpr float kDefaultRpmEnd = 5500.0f;

constexpr float kDefaultFilterAlpha = 0.2f;
constexpr float kDefaultKp = 0.073f;
constexpr float kDefaultKi = 0.524f;
constexpr float kLegacyDefaultKp = 0.02f;
constexpr float kLegacyDefaultKi = 0.05f;
constexpr bool  kDefaultUsePot3Kp = false;
constexpr float kDefaultAmbientRhOffsetPct = 0.0f;
constexpr float kDefaultMotorModeMaxRpm = 2000.0f;

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
// Conservative baseline for 100 ms control loop cadence.
float kp = kDefaultKp;
float ki = kDefaultKi;
bool usePot3Kp = kDefaultUsePot3Kp;
float ambientRhOffsetPct = kDefaultAmbientRhOffsetPct;
float motorModeMaxRpm = kDefaultMotorModeMaxRpm;

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
    kp = kDefaultKp;
    ki = kDefaultKi;
    usePot3Kp = kDefaultUsePot3Kp;
    ambientRhOffsetPct = kDefaultAmbientRhOffsetPct;
    motorModeMaxRpm = kDefaultMotorModeMaxRpm;

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
    kp = prefs.getFloat("kp", kp);
    ki = prefs.getFloat("ki", ki);
    usePot3Kp = prefs.getBool("pot3kp", usePot3Kp);
    ambientRhOffsetPct = prefs.getFloat("rhOffPct", ambientRhOffsetPct);
    motorModeMaxRpm = prefs.getFloat("motorMaxRpm", motorModeMaxRpm);

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
    prefs.putFloat("kp", kp);
    prefs.putFloat("ki", ki);
    prefs.putBool("pot3kp", usePot3Kp);
    prefs.putFloat("rhOffPct", ambientRhOffsetPct);
    prefs.putFloat("motorMaxRpm", motorModeMaxRpm);

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
