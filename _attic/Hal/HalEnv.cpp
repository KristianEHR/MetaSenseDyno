#include "HalEnv.h"

// Replace with your real BMP280/BME280 driver

static float ambC = 20.0f;
static float press = 1013.25f;
static float density = 1.204f;

namespace HalEnv {

void begin() {
    // init I2C sensor
}

void update() {
    // TODO: Replace with real sensor readings
    ambC = 20.0f;
    press = 1013.25f;

    // Simple air density model
    density = (press * 100) / (287.05f * (ambC + 273.15f));
}

float airDensity() { return density; }
float ambientC()   { return ambC; }
float pressureHpa(){ return press; }

}
