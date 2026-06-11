#include "WebSocket.h"
#include "Notify.h"
#include "Input.h"
#include "Settings.h"
#include "CalibrationTask.h"

#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

namespace MetaSense::CommandRouter {

void handleWebSocketMessage(AsyncWebSocketClient *client, const String& msg);

} // namespace MetaSense::CommandRouter

namespace {

AsyncWebSocket ws("/ws");

void handleText(AsyncWebSocketClient *client, const String& msg)
{
    MetaSense::CommandRouter::handleWebSocketMessage(client, msg);
}

void onEvent(AsyncWebSocket *server,
             AsyncWebSocketClient *client,
             AwsEventType type,
             void *arg,
             uint8_t *data,
             size_t len)
{
    if (type == WS_EVT_CONNECT) {
        MetaSense::Notify::sendStatus("WebSocket client connected");
    }
    else if (type == WS_EVT_DISCONNECT) {
        MetaSense::Notify::sendStatus("WebSocket client disconnected");
    }
    else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->opcode == WS_TEXT) {
            String msg;
            msg.reserve(len);
            for (size_t i = 0; i < len; i++) {
                msg += (char)data[i];
            }
            handleText(client, msg);
        }
    }
}

} // anonymous namespace


namespace MetaSense::WebSocket {

AsyncWebSocket& socket()
{
    return ws;   // <-- THIS is what WebServer.cpp needs
}

void begin(AsyncWebServer& server)
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

} // namespace MetaSense::WebSocket
