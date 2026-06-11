#include <ESPAsyncWebServer.h>

#include "CommandRouter.h"

namespace {

// Limit queued messages per client to 4 (default is 8).
// Lower limit means connections are closed earlier rather than
// building a large backlog that stalls the browser.
AsyncWebSocket ws("/ws");

} // anonymous namespace


namespace MetaSense::WebSocketServer {

AsyncWebSocket& socket()
{
    return ws;
}

void begin(AsyncWebServer& server)
{
    ws.onEvent([](AsyncWebSocket* socket,
                  AsyncWebSocketClient* client,
                  AwsEventType type,
                  void* arg,
                  uint8_t* data,
                  size_t len) {
        (void)socket;

        if (type != WS_EVT_DATA) {
            return;
        }

        AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
        if (info == nullptr || !info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
            return;
        }

        String msg;
        msg.reserve(len);
        for (size_t i = 0; i < len; ++i) {
            msg += static_cast<char>(data[i]);
        }

        MetaSense::CommandRouter::handleWebSocketMessage(client, msg);
    });

    server.addHandler(&ws);
}

void loop()
{
    ws.cleanupClients();
}

} // namespace MetaSense::WebSocketServer
