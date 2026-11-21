#include <Arduino.h>
#include "Globals.h"
#include "Web/WebServer.h"
#include "Dyno/DynoControl.h"
#include "Can/LeafCan.h"

using namespace MetaSense;

static DynoControl dyno;
static LeafCan     leafCan;

static uint32_t lastUpdateMs = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Start HTTP + WebSocket server
    MetaSense::WebServer::begin();

    dyno.begin();
    leafCan.begin(500000);
}

void loop() {
    uint32_t now = millis();
    float dt = (now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;

    leafCan.poll();
    dyno.update(dt);
}
