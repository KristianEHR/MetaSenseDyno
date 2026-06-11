#include "Settings.h"
#include <Preferences.h>

namespace { // LOCAL SCOPE

Preferences prefs;

float filterAlphaLocal = 0.2f;
float kpLocal          = 0.05f;
float kiLocal          = 0.10f;
float rpmTargetLocal   = 0.0f;
float tachoCalLocal    = 10.0f;

void load()
{
    prefs.begin("metasense", true);

    filterAlphaLocal = prefs.getFloat("filterAlpha", 0.2f);
    kpLocal          = prefs.getFloat("kp", 0.05f);
    kiLocal          = prefs.getFloat("ki", 0.10f);
    rpmTargetLocal   = prefs.getFloat("rpmTarget", 0.0f);
    tachoCalLocal    = prefs.getFloat("tachoCal", 10.0f);

    prefs.end();
}

void save()
{
    prefs.begin("metasense", false);

    prefs.putFloat("filterAlpha", filterAlphaLocal);
    prefs.putFloat("kp",          kpLocal);
    prefs.putFloat("ki",          kiLocal);
    prefs.putFloat("rpmTarget",   rpmTargetLocal);
    prefs.putFloat("tachoCal",    tachoCalLocal);

    prefs.end();
}

} // anonymous namespace


namespace MetaSense::Settings { // EXTERNAL SCOPE

float filterAlpha = 0.2f;
float kp          = 0.05f;
float ki          = 0.10f;

void begin()
{
    load();
    filterAlpha = filterAlphaLocal;
    kp          = kpLocal;
    ki          = kiLocal;
}

void setRpmTarget(float rpm)
{
    rpmTargetLocal = rpm;
    save();
}

float getRpmTarget()
{
    return rpmTargetLocal;
}

void setTachoCal(float cal)
{
    tachoCalLocal = cal;
    save();
}

float getTachoCal()
{
    return tachoCalLocal;
}

} // namespace MetaSense::Settings
