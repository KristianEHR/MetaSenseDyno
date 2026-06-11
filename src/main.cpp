#include <Arduino.h>
#include "controlTask.h"
#include "ModbusPublisher.h"
#include "WebSocketServer.h"

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

const char* ssid     = "5djnmv47";
const char* password = "Niser0201";

namespace {

MetaSense::ModbusPublisher modbusPublisher;
AsyncWebServer webServer(80);
bool otaStarted = false;
bool webServerStarted = false;

constexpr const char* kOtaHostname = "dyno-controller";
constexpr const char* kOtaPassword = "metasense";

bool wifiCredentialsConfigured();

const char* wifiStatusToString(wl_status_t status)
{
    switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    default: return "WL_UNKNOWN";
    }
}

void logLine(const char* message)
{
    Serial.println(message);
    Serial0.println(message);
}

void logWifiConnecting(const char* targetSsid)
{
    Serial.printf("[BOOT] WiFi connecting to SSID '%s'\n", targetSsid);
    Serial0.printf("[BOOT] WiFi connecting to SSID '%s'\n", targetSsid);
}

void logWifiConnected()
{
    const String ip = WiFi.localIP().toString();
    Serial.printf("[BOOT] WiFi connected, IP: %s\n", ip.c_str());
    Serial0.printf("[BOOT] WiFi connected, IP: %s\n", ip.c_str());
    Serial.printf("[BOOT] WiFi MAC: %s\n", WiFi.macAddress().c_str());
    Serial0.printf("[BOOT] WiFi MAC: %s\n", WiFi.macAddress().c_str());
    Serial.println("[BOOT] Web UI: http://dyno-controller.local/");
    Serial0.println("[BOOT] Web UI: http://dyno-controller.local/");
}

void setupWebServer()
{
    if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) {
        logLine("[BOOT] LittleFS mount failed");
        return;
    }

    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", "text/html");
    });
    webServer.on("/index.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", "text/html");
    });
    webServer.on("/index1.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(404, "text/plain", "index1.html is obsolete; use /index.html");
    });
    webServer.on("/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/settings.html", "text/html");
    });
    webServer.on("/trend", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/trend.html", "text/html");
    });
    webServer.on("/update", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/update.html", "text/html");
    });
    webServer.on("/update_fs", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/update_fs.html", "text/html");
    });

    webServer.serveStatic("/", LittleFS, "/");
    MetaSense::WebSocketServer::begin(webServer);
    webServer.begin();
    logLine("[BOOT] Web server ready on port 80");
}

void logWifiConfiguration()
{
    const bool credsSet = wifiCredentialsConfigured();
    Serial.println("[BOOT] WiFi config:");
    Serial0.println("[BOOT] WiFi config:");

    Serial.println("[BOOT]  - Mode: STA");
    Serial0.println("[BOOT]  - Mode: STA");

    if (credsSet) {
        Serial.printf("[BOOT]  - SSID: %s\n", ssid);
        Serial0.printf("[BOOT]  - SSID: %s\n", ssid);
    } else {
        logLine("[BOOT]  - SSID: <not configured>");
    }

    Serial.printf("[BOOT]  - OTA hostname: %s\n", kOtaHostname);
    Serial0.printf("[BOOT]  - OTA hostname: %s\n", kOtaHostname);

    const bool otaAuthConfigured = strlen(kOtaPassword) > 0;
    Serial.printf("[BOOT]  - OTA auth: %s\n", otaAuthConfigured ? "enabled" : "disabled");
    Serial0.printf("[BOOT]  - OTA auth: %s\n", otaAuthConfigured ? "enabled" : "disabled");
}

bool wifiCredentialsConfigured()
{
    return strlen(ssid) > 0 && strcmp(ssid, "YOUR_SSID") != 0;
}

void setupWifi()
{
    if (!wifiCredentialsConfigured()) {
        logLine("[BOOT] WiFi credentials not configured, OTA disabled");
        return;
    }

    WiFi.mode(WIFI_STA);

    // Disable power-save: prevents the radio from sleeping between DTIM beacons,
    // which is the most common cause of random disconnects on ESP32.
    WiFi.setSleep(false);

    // Maximum TX power for best range / stable association.
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    // Auto-reconnect on disconnect without rebooting.
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false); // don't hammer NVS on every reconnect

    // Register disconnect/reconnect event for serial diagnostics.
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        switch (event) {
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.printf("[WiFi] Disconnected, reason: %d — reconnecting...\n",
                          info.wifi_sta_disconnected.reason);
            Serial0.printf("[WiFi] Disconnected, reason: %d — reconnecting...\n",
                           info.wifi_sta_disconnected.reason);
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("[WiFi] Associated to AP");
            Serial0.println("[WiFi] Associated to AP");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.printf("[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
            Serial0.printf("[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
            break;
        default:
            break;
        }
    });

    WiFi.begin(ssid, password);
    logWifiConnecting(ssid);
}

void setupOtaOnceConnected()
{
    if (otaStarted || WiFi.status() != WL_CONNECTED) {
        return;
    }

    ArduinoOTA.setHostname(kOtaHostname);
    ArduinoOTA.setPassword(kOtaPassword);

    ArduinoOTA.onStart([]() {
        logLine("[OTA] Start");
    });

    ArduinoOTA.onEnd([]() {
        logLine("[OTA] End");
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error: %u\n", static_cast<unsigned>(error));
        Serial0.printf("[OTA] Error: %u\n", static_cast<unsigned>(error));
    });

    ArduinoOTA.begin();
    otaStarted = true;
    logWifiConnected();
    logLine("[BOOT] OTA ready");

    // Attempt NTP time sync (non-blocking; sync happens in background)
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    logLine("[BOOT] NTP sync requested");

    if (!webServerStarted) {
        setupWebServer();
        webServerStarted = true;
    }
}

} // anonymous namespace

void setup()
{
    Serial0.begin(115200);
    Serial.begin(115200);
    delay(200);
    logLine("[BOOT] MetaSense startup");
    logWifiConfiguration();

    setupWifi();
    MetaSense::ControlTask::begin();
    if (!modbusPublisher.begin()) {
        logLine("[BOOT] ModbusPublisher begin failed");
    }

    logLine("[BOOT] setup complete");
}

void loop()
{
    setupOtaOnceConnected();
    if (otaStarted) {
        ArduinoOTA.handle();
    }

    MetaSense::WebSocketServer::loop();

    MetaSense::ControlTask::loop();
    modbusPublisher.update();

    static uint32_t lastStatusMs = 0;
    const uint32_t now = millis();
    if (now - lastStatusMs > 3000) {
        lastStatusMs = now;
        if (wifiCredentialsConfigured()) {
            const wl_status_t status = WiFi.status();
            const String ip = WiFi.localIP().toString();
            Serial.printf("[BOOT] WiFi status: %d (%s), board IP: %s, OTA: %s\n",
                          static_cast<int>(status),
                          wifiStatusToString(status),
                          ip.c_str(),
                          otaStarted ? "ready" : "not-ready");
            Serial0.printf("[BOOT] WiFi status: %d (%s), board IP: %s, OTA: %s\n",
                           static_cast<int>(status),
                           wifiStatusToString(status),
                           ip.c_str(),
                           otaStarted ? "ready" : "not-ready");
        }
    }
}
