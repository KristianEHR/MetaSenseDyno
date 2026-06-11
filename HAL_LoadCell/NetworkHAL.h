#pragma once
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

class NetworkHAL {
public:
    bool begin(const char* ssid, const char* password);
    AsyncWebServer& server();

private:
    AsyncWebServer web{80};
};
