#include "HalLoadcell.h"
#include "Settings.h"

// Replace with your real HX711 or ADC implementation
static float offset = 0;

namespace HalLoadcell {

void begin() {
    offset = 0;
}

void tare() {
    offset = readRaw();
}

float readRaw() {
    // TODO: Replace with real ADC/HX711 reading
    return 0.0f;
}

float readKg() {
    float raw = readRaw() - offset;
    return raw * MetaSense::Settings::calFactor;

}

}
