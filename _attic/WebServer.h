#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>

namespace MetaSense::WebServer {

    void begin();
    AsyncWebServer& server();

}
