#include "WebServer.h"
#include "../Globals.h"
#include "../Web/WebSocket.h"

namespace MetaSense::WebServer {

void begin()
{
    // Attach WebSocket to server
    MetaSense::WebSocket::begin(MetaSense::server, MetaSense::ws);

    // Basic root page
    MetaSense::server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/plain", "MetaSense Dyno System");
    });

    // Start server
    MetaSense::server.begin();
}

}
