#include "HalNetwork.h"

// You can move these to a config header if you like
static AsyncWebServer g_server(80);
static AsyncWebSocket g_ws("/ws");

namespace HalNetwork {

    AsyncWebServer &server() {
        return g_server;
    }

    AsyncWebSocket &websocket() {
        return g_ws;
    }

    void begin(const char *ssid, const char *password) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);

        // Simple blocking connect; you can replace with non-blocking later
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
            delay(250);
        }

        // Attach WebSocket to server
        g_ws.enable(true);
        g_server.addHandler(&g_ws);

        // Minimal root handler (you probably already serve files from LittleFS)
        g_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
            request->send(200, "text/plain", "MetaSense DYNO online");
        });

        g_server.begin();
    }

    void loop() {
        // Currently nothing needed; AsyncWebServer + AsyncTCP are event-driven.
        // Keep this for future time-based network tasks.
    }

} // namespace HalNetwork
