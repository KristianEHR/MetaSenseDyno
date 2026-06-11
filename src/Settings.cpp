#include "Settings.h"
#include <math.h>

namespace {

float rpmTargetLocal = 0.0f;
float tachoCalLocal = 10.0f;
float rpmStartLocal = 1500.0f;
float rpmEndLocal = 5500.0f;

} // anonymous namespace


namespace MetaSense::Settings {

float filterAlpha = 0.2f;
float kp = 0.05f;
float ki = 0.10f;

// Gauge display ranges
float maxRPM          = 18000.0f;
float maxHP           = 25.0f;
float maxTorque       = 200.0f;
float armCm           = 20.0f;

// Pulse inputs
float pulsesPerRev     = 1.0f;
float pulsesPerRevDrum = 1.0f;

// Drivetrain efficiency
float drivetrainEff    = 95.0f;

// Inertia dyno parameters (defaults)
bool  inertiaMode     = false;
bool  useCanLeafRpm   = true;
float drumMassKg      = 10.0f;   // kg
float drumRadiusM     = 0.15f;   // 30 cm diameter → 0.15 m radius
float drumWallM       = 0.0f;    // 0 = solid cylinder
float drumInertiaKgM2 = 0.1125f; // J = 0.5 * 10 * 0.15^2 = 0.1125 kg·m²
float virtGearRatio   = 1.0f;    // drum→engine ratio

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
