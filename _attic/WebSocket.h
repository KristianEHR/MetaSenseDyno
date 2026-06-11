#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>

namespace MetaSense::WebSocket {

    AsyncWebSocket& socket();
    void begin(AsyncWebServer& server);

}
