#pragma once

namespace MetaSense::Settings {

extern float filterAlpha;
extern float kp;
extern float ki;

// Gauge display ranges (sent back in settings snapshot)
extern float maxRPM;
extern float maxHP;
extern float maxTorque;
extern float armCm;

// Pulse inputs
extern float pulsesPerRev;     // engine hall
extern float pulsesPerRevDrum; // drum/roller hall

// Drivetrain efficiency (0..100 %)
extern float drivetrainEff;
// Inertia dyno parameters
extern bool  inertiaMode;       // true when dynoMode == "inertia"
extern bool  useCanLeafRpm;      // true=Leaf CAN RPM, false=tachogen analog RPM
extern float drumMassKg;        // total mass of drum/flywheel, kg
extern float drumRadiusM;       // outer radius of drum, metres
extern float drumWallM;         // wall thickness for hollow cylinder, metres (0 = solid)
extern float drumInertiaKgM2;   // effective moment of inertia J, kg·m²  (recomputed on change)
extern float virtGearRatio;     // drum RPM × ratio = engine RPM (1.0 = direct)

void setDrumParams(float massKg, float radiusM, float wallM);
void setDrumInertiaCustom(float J);
void setInertiaMode(bool enabled);

void setRpmTarget(float rpm);
float getRpmTarget();

void setRpmStart(float rpm);
float rpmStart();

void setRpmEnd(float rpm);
float rpmEnd();

void setTachoCal(float cal);
float getTachoCal();

} // namespace MetaSense::Settings
