#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

namespace HalNetwork {

    // Initialize WiFi + AsyncWebServer + WebSocket
    void begin(const char *ssid, const char *password);

    // Call from main loop (for future time-based tasks if needed)
    void loop();

    // Accessors so other modules can register routes / broadcast
    AsyncWebServer &server();
    AsyncWebSocket &websocket();

} // namespace HalNetwork
