#include <ESPAsyncWebServer.h>
#include <map>

#include "CommandRouter.h"
#include "TelnetSerialBridge.h"

namespace {

AsyncWebSocket ws("/ws");
std::map<uint32_t, String> rxBuffers;

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

        if (client == nullptr) {
            return;
        }

        if (type == WS_EVT_CONNECT) {
            Serial.printf("[WS] Client connected: id=%u, ip=%s\n", client->id(), client->remoteIP().toString().c_str());
            MetaSense::TelnetSerialBridge::telnetBridgePrintf(
                "[WS] Client connected: id=%u, ip=%s\n", client->id(), client->remoteIP().toString().c_str());
            return;
        }

        if (type == WS_EVT_DISCONNECT) {
            Serial.printf("[WS] Client disconnected: id=%u\n", client->id());
            MetaSense::TelnetSerialBridge::telnetBridgePrintf("[WS] Client disconnected: id=%u\n", client->id());
            rxBuffers.erase(client->id());
            return;
        }

        if (type != WS_EVT_DATA) {
            return;
        }

        AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
        if (info == nullptr || info->opcode != WS_TEXT) {
            return;
        }

        String& msg = rxBuffers[client->id()];
        if (info->index == 0) {
            msg = "";
            msg.reserve(info->len);
        }

        for (size_t i = 0; i < len; ++i) {
            msg += static_cast<char>(data[i]);
        }

        if (info->final) {
            MetaSense::CommandRouter::handleWebSocketMessage(client, msg);
            rxBuffers.erase(client->id());
        }
    });

    server.addHandler(&ws);
}

void loop()
{
    // Send ping to all connected clients to keep connections alive
    static uint32_t lastPingMs = 0;
    static uint32_t lastDiagnosticsMs = 0;
    const uint32_t now = millis();
    if (now - lastPingMs >= 5000U) {  // Ping every 5 seconds
        lastPingMs = now;
        ws.pingAll();
    }
    
    // Diagnostics: monitor client buffer health every 10 seconds to preempt disconnections
    if (now - lastDiagnosticsMs >= 10000U) {
        lastDiagnosticsMs = now;
        size_t connectedCount = 0;
        size_t warningCount = 0;
        for (auto &client : ws.getClients()) {
            if (client.status() == WS_CONNECTED) {
                connectedCount++;
                // Check if client's message queue is approaching limits
                // ESPAsyncWebServer's default queue can be small; high queue usage indicates
                // browser may not be keeping pace with 50ms telemetry rate
                size_t queueLen = client.queueLen();
                if (queueLen > 3) {  // Warn if 3+ messages queued (potential bottleneck)
                    warningCount++;
                }
            }
        }
        if (connectedCount > 0 && warningCount > 0) {
            Serial.printf("[WS] Diagnostics: %u connected, %u clients with queue backlog (3+ msgs queued)\n",
                         connectedCount, warningCount);
        }
    }
    
    ws.cleanupClients();
}

} // namespace MetaSense::WebSocketServer
