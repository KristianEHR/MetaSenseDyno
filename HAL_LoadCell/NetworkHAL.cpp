#include "NetworkHAL.h"

bool NetworkHAL::begin(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
        return false;

    web.begin();
    return true;
}

AsyncWebServer& NetworkHAL::server() {
    return web;
}
