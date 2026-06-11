#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>

namespace MetaSense::WebSocket {

    // Initialize the WebSocket and attach event handlers
    void begin(AsyncWebServer &server, AsyncWebSocket &ws);

    // Broadcast a text message to all connected clients
    void broadcast(const String &msg);

}
