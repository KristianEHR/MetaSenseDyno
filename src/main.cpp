#include <Arduino.h>
#include "controlTask.h"
#include "CommandRouter.h"
#include "Input.h"
#include "ModbusPublisher.h"
#include "RunStorage.h"
#include "Settings.h"
#include "WebSocketServer.h"

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>

#include "globals.h"

const char* ssid     = "5djnmv47";
const char* password = "Niser0201";

namespace {
 
MetaSense::ModbusPublisher modbusPublisher;
AsyncWebServer webServer(80);
bool otaStarted = false;
bool webServerStarted = false;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t networkTaskHandle = nullptr;
TaskHandle_t modbusTaskHandle = nullptr;

constexpr const char* kOtaHostname = "dyno-controller";
constexpr const char* kOtaPassword = "metasense";
constexpr uint32_t kI2cClockHz = 25000;
constexpr uint16_t kI2cTimeoutMs = 50;
constexpr uint32_t kControlPeriodMs = 100;
constexpr uint32_t kModbusPeriodMs = 50;
constexpr uint32_t kHeartbeatPeriodMs = 5000;
constexpr UBaseType_t kControlTaskPriority = 5;
constexpr UBaseType_t kNetworkTaskPriority = 3;
constexpr UBaseType_t kModbusTaskPriority = 1;

bool wifiCredentialsConfigured();

void scanI2cBus()
{
    uint8_t found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        const uint8_t err = Wire.endTransmission();
        if (err == 0) {
            ++found;
            Serial.printf("[BOOT] I2C device found @ 0x%02X\n", addr);
            Serial0.printf("[BOOT] I2C device found @ 0x%02X\n", addr);
        }
    }

    if (found == 0) {
        Serial.println("[BOOT] I2C scan: no devices found");
        Serial0.println("[BOOT] I2C scan: no devices found");
    } else {
        Serial.printf("[BOOT] I2C scan complete: %u device(s)\n", found);
        Serial0.printf("[BOOT] I2C scan complete: %u device(s)\n", found);
    }
}

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

void controlTaskEntry(void* /*parameter*/)
{
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        MetaSense::ControlTask::loop();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

void modbusTaskEntry(void* /*parameter*/)
{
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        modbusPublisher.update();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kModbusPeriodMs));
    }
}

void networkTaskEntry(void* /*parameter*/)
{
    uint32_t lastStatusMs = 0;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25));

        setupOtaOnceConnected();

        if (otaStarted) {
            ArduinoOTA.handle();
        }

        MetaSense::WebSocketServer::loop();
        MetaSense::Input::publish();

        const uint32_t now = millis();
        if (now - lastStatusMs > kHeartbeatPeriodMs) {
            lastStatusMs = now;
            if (wifiCredentialsConfigured()) {
                const wl_status_t status = WiFi.status();
                const String ip = WiFi.localIP().toString();
                Serial.printf("[HEARTBEAT] ssid=%s, wifi=%d (%s), ip=%s, ota=%s\n",
                              ssid,
                              static_cast<int>(status),
                              wifiStatusToString(status),
                              ip.c_str(),
                              otaStarted ? "ready" : "not-ready");
                Serial0.printf("[HEARTBEAT] ssid=%s, wifi=%d (%s), ip=%s, ota=%s\n",
                               ssid,
                               static_cast<int>(status),
                               wifiStatusToString(status),
                               ip.c_str(),
                               otaStarted ? "ready" : "not-ready");
            }
        }

    }
}

} // anonymous namespace

void setup()
{
    Serial.begin(115200);
    Serial0.begin(115200);
    delay(200);
    logLine("[BOOT] MetaSense startup");
    Wire.begin(MetaSense::Globals::kI2cSdaPin, MetaSense::Globals::kI2cSclPin);
    Wire.setClock(kI2cClockHz);
    Wire.setTimeOut(kI2cTimeoutMs);
    Serial.printf("[BOOT] I2C ready (SDA=%d, SCL=%d)\n",
                  MetaSense::Globals::kI2cSdaPin,
                  MetaSense::Globals::kI2cSclPin);
    Serial0.printf("[BOOT] I2C ready (SDA=%d, SCL=%d)\n",
                   MetaSense::Globals::kI2cSdaPin,
                   MetaSense::Globals::kI2cSclPin);
    Serial.printf("[BOOT] I2C timing: %lu Hz, timeout=%u ms\n",
                  static_cast<unsigned long>(kI2cClockHz),
                  static_cast<unsigned>(kI2cTimeoutMs));
    Serial0.printf("[BOOT] I2C timing: %lu Hz, timeout=%u ms\n",
                   static_cast<unsigned long>(kI2cClockHz),
                   static_cast<unsigned>(kI2cTimeoutMs));
    MetaSense::Settings::loadFromStorage();
    Serial.println("[BOOT] Settings loaded from storage (if available)");
    Serial0.println("[BOOT] Settings loaded from storage (if available)");
    scanI2cBus();
    logWifiConfiguration();

    MetaSense::ControlTask::begin();
    if (MetaSense::CommandRouter::loadFactoryProfileOnBoot()) {
        logLine("[BOOT] Factory profile loaded from FS");
    } else {
        logLine("[BOOT] No factory profile found in FS");
    }
    setupWifi();
    if (!modbusPublisher.begin()) {
        logLine("[BOOT] ModbusPublisher begin failed");
    }

    BaseType_t controlCreated = xTaskCreatePinnedToCore(
        controlTaskEntry,
        "controlTask",
        8192,
        nullptr,
        kControlTaskPriority,
        &controlTaskHandle,
        1);

    BaseType_t networkCreated = xTaskCreatePinnedToCore(
        networkTaskEntry,
        "networkTask",
        12288,
        nullptr,
        kNetworkTaskPriority,
        &networkTaskHandle,
        0);

    BaseType_t modbusCreated = xTaskCreatePinnedToCore(
        modbusTaskEntry,
        "modbusTask",
        8192,
        nullptr,
        kModbusTaskPriority,
        &modbusTaskHandle,
        0);

    if (controlCreated != pdPASS) {
        logLine("[BOOT] Failed to start control task");
    }

    if (networkCreated != pdPASS) {
        logLine("[BOOT] Failed to start network task");
    }

    if (modbusCreated != pdPASS) {
        logLine("[BOOT] Failed to start modbus task");
    }

    MetaSense::RunStorage::setPublishTaskHandle(networkTaskHandle);

    logLine("[BOOT] setup complete");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
