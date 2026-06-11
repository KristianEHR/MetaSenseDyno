#include "WebSocket.h"
#include "../Globals.h"

namespace MetaSense::WebSocket {

static void onEvent(AsyncWebSocket *server,
                    AsyncWebSocketClient *client,
                    AwsEventType type,
                    void *arg,
                    uint8_t *data,
                    size_t len)
{
    if (type == WS_EVT_CONNECT) {
        Serial.println("WebSocket client connected");
    }
    else if (type == WS_EVT_DISCONNECT) {
        Serial.println("WebSocket client disconnected");
    }
    else if (type == WS_EVT_DATA) {
        // Incoming messages can be handled here if needed
    }
}

void begin(AsyncWebServer &server, AsyncWebSocket &ws)
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

void broadcast(const String &msg)
{
    MetaSense::ws.textAll(msg);
}

}
