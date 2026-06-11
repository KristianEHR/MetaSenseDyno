#pragma once
#include <Arduino.h>

namespace MetaSense::Settings {

    // Persistent configuration
    extern float filterAlpha;
    extern float kp;
    extern float ki;

    extern float calFactor;
    extern int   maxRPM;
    extern float maxHP;
    extern float maxTorque;
    extern float armCm;

    extern float pulsesPerRev;
    extern float rpmFilter;

    extern String mode;             // "inertia" or "brake"
    extern float virtGearRatio;

    extern float drumMass;
    extern float drumCm;
    extern float pulsesPerRevDrum;
    extern String drumInertiaType;  // "solid", "hollow", "custom"
    extern float drumWallCm;
    extern float drumInertiaCustom;

    extern float brakeToEngineRatio;
    extern float drivetrainEff;
    extern float armLengthM;
    extern String rpmSource;        // "spark", "drum", "brake", "cvt"

    // Version info
    extern String versionString;
    extern String buildDateString;

    // API
    void begin();
    void setRpmTarget(float rpm);
    float getRpmTarget();

    void setTachoCal(float cal);
    float getTachoCal();

    void save();
    void resetAll();
    void setSingle(const String& key, const String& value);

    // WiFi
    void setWifiClient(const String& ssid, const String& pass);
    void resetToApMode();
}
