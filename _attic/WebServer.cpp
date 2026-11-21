#include "WebServer.h"
#include "WebSocket.h"
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

namespace { // LOCAL SCOPE

AsyncWebServer serverInst(80);

void setupStatic()
{
    serverInst.serveStatic("/", SPIFFS, "/index.html");
    serverInst.serveStatic("/settings.html", SPIFFS, "/settings.html");
    serverInst.serveStatic("/css/", SPIFFS, "/css/");
    serverInst.serveStatic("/js/", SPIFFS, "/js/");
}

} // anonymous namespace


namespace MetaSense::WebServer { // EXTERNAL SCOPE

AsyncWebServer& server()
{
    return serverInst;
}

void begin()
{
    SPIFFS.begin(true);
    setupStatic();

    // Attach WebSocket handler from WebSocket.cpp
    serverInst.addHandler(&MetaSense::WebSocket::socket());

    serverInst.begin();
    Serial.println("[WebServer] HTTP server started");
}

} // namespace MetaSense::WebServer
