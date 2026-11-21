#include <Arduino.h>
#include "Dyno/DynoControl.h"

static MetaSense::DynoControl g_dyno;

void setup() {
    Wire.begin();
    g_dyno.begin();
}

void loop() {
    static uint32_t lastMs = millis();
    uint32_t now = millis();
    float dtSec = (now - lastMs) / 1000.0f;
    lastMs = now;

    g_dyno.update(dtSec);
}
