#include <Arduino.h>
#include "controlTask.h"
#include "CommandRouter.h"
#include "Input.h"
#include "ModbusPublisher.h"
#include "RunStorage.h"
#include "Settings.h"
#include "WebSocketServer.h"
#include "MetaSenseDynoVCU.h"

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <time.h>

#include "globals.h"
#include "I2cBusLock.h"

// Enable/disable runtime task instrumentation (set to 0 to disable)
#define ENABLE_RUNTIME_INSTRUMENTATION 0

// VCU ready source switch:
// 1 = use GPIO RB+ input (normal operation)
// 0 = force VCU ready true (bench testing without VCU)
#if !defined(VCU_switch) && defined(VCU_set)
#define VCU_switch VCU_set
#endif

#ifndef VCU_switch
#define VCU_switch 0
#endif

#ifndef METASENSE_VCU_OWNS_HV_RPLUS_PRECHARGE
#define METASENSE_VCU_OWNS_HV_RPLUS_PRECHARGE 0
#endif

#ifndef METASENSE_VCU_SIM_MODE
#define METASENSE_VCU_SIM_MODE 0
#endif

namespace MetaSense::HardwareOutputStateMachine {
void setVcuRelayOverride(bool enabled, bool rPlus, bool precharge);
}

namespace MetaSense::Globals {
const bool kVcuSwitch = (VCU_switch != 0);
}

const char* ssid     = "5djnmv47";
const char* password = "Niser0201";

namespace {
 
MetaSense::ModbusPublisher modbusPublisher;
MetaSense::MetaSenseDynoVCU dynoVcu;
AsyncWebServer webServer(80);
bool otaStarted = false;
bool webServerStarted = false;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t networkTaskHandle = nullptr;
TaskHandle_t modbusTaskHandle = nullptr;

constexpr const char* kOtaHostname = "dyno-controller";
constexpr const char* kOtaPassword = "metasense";
constexpr uint32_t kI2cClockHz = 50000;
constexpr uint16_t kI2cTimeoutMs = 50;
#ifndef METASENSE_CONTROL_PERIOD_MS
#define METASENSE_CONTROL_PERIOD_MS 10
#endif
#ifndef METASENSE_HEARTBEAT_PERIOD_MS
#define METASENSE_HEARTBEAT_PERIOD_MS 5000
#endif
constexpr uint32_t kControlPeriodMs = METASENSE_CONTROL_PERIOD_MS;
constexpr uint32_t kModbusPeriodMs = 50;
constexpr uint32_t kHeartbeatPeriodMs = METASENSE_HEARTBEAT_PERIOD_MS;
constexpr UBaseType_t kControlTaskPriority = 5;
constexpr UBaseType_t kNetworkTaskPriority = 3;
constexpr UBaseType_t kModbusTaskPriority = 1;

struct TaskRuntimeStats {
    volatile uint32_t lastUs = 0;
    volatile uint32_t maxUs = 0;
    volatile uint32_t emaUs = 0;
    volatile uint32_t loops = 0;
    volatile uint32_t overruns = 0;
};

TaskRuntimeStats controlTaskStats;
TaskRuntimeStats networkTaskStats;
TaskRuntimeStats modbusTaskStats;

void updateBenchVcuSimInputs(uint32_t nowMs,
                             bool prechargeCmd,
                             bool rPlusCmd,
                             bool& inverter12vOn,
                             float& hvVoltage)
{
    static bool initialized = false;
    static uint32_t lastMs = 0;
    static bool simInv12v = false;
    static float simHvVoltage = 0.0f;

    if (!initialized) {
        initialized = true;
        lastMs = nowMs;
        simInv12v = false;
        simHvVoltage = 0.0f;
    }

    uint32_t deltaMs = nowMs - lastMs;
    if (deltaMs > 250U) {
        deltaMs = 250U;
    }
    lastMs = nowMs;

    const float dt = static_cast<float>(deltaMs) / 1000.0f;

    if (nowMs > 1200U) {
        simInv12v = true;
    }

    if (!simInv12v) {
        simHvVoltage = 0.0f;
    } else if (prechargeCmd && !rPlusCmd) {
        simHvVoltage += 220.0f * dt;
    } else if (rPlusCmd) {
        simHvVoltage += 80.0f * dt;
    } else {
        simHvVoltage -= 140.0f * dt;
    }

    if (simHvVoltage < 0.0f) {
        simHvVoltage = 0.0f;
    }
    if (simHvVoltage > 360.0f) {
        simHvVoltage = 360.0f;
    }

    inverter12vOn = simInv12v;
    hvVoltage = simHvVoltage;
}

void recordTaskRuntime(TaskRuntimeStats& stats, uint32_t elapsedUs, uint32_t periodUs)
{
    stats.lastUs = elapsedUs;
    if (elapsedUs > stats.maxUs) {
        stats.maxUs = elapsedUs;
    }
    if (stats.emaUs == 0) {
        stats.emaUs = elapsedUs;
    } else {
        stats.emaUs = (stats.emaUs * 7U + elapsedUs) / 8U;
    }
    ++stats.loops;
    if (elapsedUs > periodUs) {
        ++stats.overruns;
    }
}

void resetMaxStats()
{
    controlTaskStats.maxUs = 0;
    networkTaskStats.maxUs = 0;
    modbusTaskStats.maxUs = 0;
    MetaSense::Input::resetLoadCellSamplerMaxRuntime();
}

bool wifiCredentialsConfigured();

uint32_t heartbeatTimestamp()
{
    time_t now = time(nullptr);
    return (now > 1000000000L) ? (uint32_t)now : (uint32_t)(millis() / 1000);
}

void scanI2cBus()
{
    uint8_t found = 0;
    if (MetaSense::I2cBus::take(pdMS_TO_TICKS(50))) {
        for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
            Wire.beginTransmission(addr);
            const uint8_t err = Wire.endTransmission();
            if (err == 0) {
                ++found;
                Serial.printf("[BOOT] I2C device found @ 0x%02X\n", addr);
                Serial0.printf("[BOOT] I2C device found @ 0x%02X\n", addr);
            }
        }
        MetaSense::I2cBus::give();
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
    webServer.on("/captures", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/captures.html", "text/html");
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

    auto sendJsonNoCache = [](AsyncWebServerRequest* request, int code, const String& body) {
        AsyncWebServerResponse* response = request->beginResponse(code, "application/json", body);
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
    };

    webServer.on("/api/fs/selftest", HTTP_GET, [sendJsonNoCache](AsyncWebServerRequest* request) {
        const String path = "/fs_selftest.csv";
        const String expected = "timestamp_us,source,raw_value,filtered_value\n1000,selftest,1.234,1.200\n2000,selftest,2.345,2.300\n";

        File writer = LittleFS.open(path, "w");
        if (!writer) {
            sendJsonNoCache(request, 500, "{\"ok\":false,\"msg\":\"open-write-failed\"}");
            return;
        }
        writer.print(expected);
        writer.close();

        File reader = LittleFS.open(path, "r");
        if (!reader) {
            sendJsonNoCache(request, 500, "{\"ok\":false,\"msg\":\"open-read-failed\"}");
            return;
        }
        const String text = reader.readString();
        reader.close();

        Serial.println("[FS-SELFTEST] Readback follows:");
        Serial.print(text);
        Serial0.println("[FS-SELFTEST] Readback follows:");
        Serial0.print(text);

        if (text != expected) {
            sendJsonNoCache(request, 500, "{\"ok\":false,\"msg\":\"readback-mismatch\"}");
            return;
        }

        String body = "{\"ok\":true,\"path\":\"" + path + "\",\"bytes\":" + String(text.length()) + ",\"text\":\"";
        for (size_t i = 0; i < text.length(); ++i) {
            const char c = text[i];
            if (c == '\\') body += "\\\\";
            else if (c == '"') body += "\\\"";
            else if (c == '\n') body += "\\n";
            else if (c == '\r') body += "\\r";
            else body += c;
        }
        body += "\"}";
        sendJsonNoCache(request, 200, body);
    });

    auto startFsProbeHandler = [sendJsonNoCache](AsyncWebServerRequest* request) {
        uint32_t durationMs = 10000;
        if (request->hasParam("duration_ms")) {
            const uint32_t requested = static_cast<uint32_t>(request->getParam("duration_ms")->value().toInt());
            if (requested > 0) {
                durationMs = requested;
            }
        }

        String filePath;
        if (!MetaSense::RunStorage::startFsLiveProbe(durationMs, filePath)) {
            sendJsonNoCache(request, 409, "{\"ok\":false,\"msg\":\"probe already active or start failed\"}");
            return;
        }

        sendJsonNoCache(request,
                        200,
                        "{\"ok\":true,\"msg\":\"probe started\",\"file\":\"" + filePath + "\",\"durationMs\":" + String(durationMs) + "}");
    };
    webServer.on("/api/fs/probe/start", HTTP_GET, startFsProbeHandler);
    webServer.on("/api/fs/probe/start", HTTP_POST, startFsProbeHandler);

    webServer.on("/api/fs/probe/state", HTTP_GET, [sendJsonNoCache](AsyncWebServerRequest* request) {
        sendJsonNoCache(request, 200, MetaSense::RunStorage::fsLiveProbeStateJson());
    });

    webServer.on("/api/fs/probe/read", HTTP_GET, [](AsyncWebServerRequest* request) {
        const String text = MetaSense::RunStorage::fsLiveProbeReadText();
        if (text.isEmpty()) {
            request->send(404, "text/plain", "no probe file available");
            return;
        }
        AsyncWebServerResponse* response = request->beginResponse(200, "text/plain", text);
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
    });

    webServer.on("/api/fs/probe/verify", HTTP_GET, [sendJsonNoCache](AsyncWebServerRequest* request) {
        sendJsonNoCache(request, 200, MetaSense::RunStorage::fsLiveProbeVerifyJson());
    });

    webServer.on("/api/captures/list", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncWebServerResponse* response = request->beginResponse(200,
                                                                  "application/json",
                                                                  MetaSense::RunStorage::listRawCaptures());
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
    });

    webServer.on("/api/captures/file", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (!request->hasParam("name")) {
            request->send(400, "text/plain", "missing name");
            return;
        }

        String name = request->getParam("name")->value();
        if (name.indexOf('/') >= 0 || name.indexOf("..") >= 0) {
            request->send(400, "text/plain", "invalid name");
            return;
        }

        if (!name.endsWith(".csv")) {
            name += ".csv";
        }

        String path = String("/captures/") + name;
        if (!LittleFS.exists(path)) {
            path = String("/") + name;
            if (!LittleFS.exists(path)) {
                request->send(404, "text/plain", "capture not found");
                return;
            }
        }

        request->send(LittleFS, path, "text/csv");
    });

    webServer.on("/api/captures/report", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (!request->hasParam("name")) {
            AsyncWebServerResponse* response = request->beginResponse(400, "application/json", "{}");
            response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
            response->addHeader("Pragma", "no-cache");
            response->addHeader("Expires", "0");
            request->send(response);
            return;
        }

        String name = request->getParam("name")->value();
        AsyncWebServerResponse* response = request->beginResponse(200,
                                                                  "application/json",
                                                                  MetaSense::RunStorage::loadRawCaptureReport(name));
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
    });

    auto startCaptureHandler = [](AsyncWebServerRequest* request) {
        String filePath;
        if (!MetaSense::RunStorage::startRawCapture(20000, 0.0f, 3.404f, filePath)) {
            AsyncWebServerResponse* response = request->beginResponse(409,
                                                                      "application/json",
                                                                      "{\"ok\":false,\"msg\":\"Capture already active or start failed\"}");
            response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
            response->addHeader("Pragma", "no-cache");
            response->addHeader("Expires", "0");
            request->send(response);
            return;
        }

        AsyncWebServerResponse* response = request->beginResponse(200,
                                                                  "application/json",
                                                                  "{\"ok\":true,\"msg\":\"Capture started\",\"file\":\"" + filePath + "\"}");
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
    };
    webServer.on("/api/captures/start", HTTP_POST, startCaptureHandler);

    webServer.on("/api/captures/state", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncWebServerResponse* response = request->beginResponse(200,
                                                                  "application/json",
                                                                  MetaSense::RunStorage::rawCaptureStateJson());
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
    });

    webServer.on("/api/captures/verify", HTTP_GET, [](AsyncWebServerRequest* request) {
        String name;
        if (request->hasParam("name")) {
            name = request->getParam("name")->value();
        }
        AsyncWebServerResponse* response = request->beginResponse(200,
                                                                  "application/json",
                                                                  MetaSense::RunStorage::verifyRawCaptureCsv(name));
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
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
        const uint32_t startedUs = micros();
        MetaSense::ControlTask::loop();
        const uint32_t nowMs = millis();
        bool inverter12vInput = MetaSense::Input::isVcuReady();
        float hvVoltageInput = 0.0f;

    #if METASENSE_VCU_SIM_MODE
        updateBenchVcuSimInputs(nowMs,
                    dynoVcu.getPrecharge(),
                    dynoVcu.getRPlus(),
                    inverter12vInput,
                    hvVoltageInput);
    #endif

        dynoVcu.update(nowMs,
                   MetaSense::Settings::getRpmTarget(),
                   MetaSense::Input::rpm(),
                   hvVoltageInput,
                   inverter12vInput);

        MetaSense::Input::updateVcuDebug(METASENSE_VCU_SIM_MODE != 0,
                         inverter12vInput,
                         hvVoltageInput,
                         dynoVcu.getTorqueDemand(),
                         dynoVcu.getRPlus(),
                         dynoVcu.getPrecharge(),
                         dynoVcu.getSSR(),
                         dynoVcu.getRMinus());

    #if METASENSE_VCU_OWNS_HV_RPLUS_PRECHARGE
        MetaSense::HardwareOutputStateMachine::setVcuRelayOverride(true,
                                        dynoVcu.getRPlus(),
                                        dynoVcu.getPrecharge());
    #else
        MetaSense::HardwareOutputStateMachine::setVcuRelayOverride(false, false, false);
    #endif
#if ENABLE_RUNTIME_INSTRUMENTATION
        recordTaskRuntime(controlTaskStats,
                          micros() - startedUs,
                          kControlPeriodMs * 1000U);
#endif
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

void modbusTaskEntry(void* /*parameter*/)
{
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        const uint32_t startedUs = micros();
        modbusPublisher.update();
#if ENABLE_RUNTIME_INSTRUMENTATION
        recordTaskRuntime(modbusTaskStats,
                          micros() - startedUs,
                          kModbusPeriodMs * 1000U);
#endif
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kModbusPeriodMs));
    }
}

void networkTaskEntry(void* /*parameter*/)
{
    uint32_t lastStatusMs = 0;
    bool bootStatusLogged = false;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25));
        const uint32_t startedUs = micros();

        setupOtaOnceConnected();

        if (otaStarted) {
            ArduinoOTA.handle();
        }

        MetaSense::WebSocketServer::loop();
        MetaSense::Input::publish();

#if ENABLE_RUNTIME_INSTRUMENTATION
        recordTaskRuntime(networkTaskStats,
                          micros() - startedUs,
                          25000U);
#endif

        const uint32_t now = millis();
        if (!bootStatusLogged && now >= 3000U) {
            bootStatusLogged = true;
            const bool vcuReady = MetaSense::Input::isVcuReady();
            bool nauLdoConfigured = false;
            bool nauInternalCalOk = false;
            uint8_t nauInternalCalAttempts = 0;
            MetaSense::Input::getLoadCellInitStatus(nauLdoConfigured,
                                                    nauInternalCalOk,
                                                    nauInternalCalAttempts);
            const int rbPlusLevel = digitalRead(MetaSense::Globals::kRbPlusInputPin);
            Serial.printf("[BOOTSTATUS] vcu_mode=%s, vcu_ready=%d, rb_plus=%d, nau_ldo=%d, nau_cal=%d, nau_cal_attempts=%u\n",
                          MetaSense::Globals::kVcuSwitch ? "gpio" : "forced",
                          vcuReady ? 1 : 0,
                          rbPlusLevel,
                          nauLdoConfigured ? 1 : 0,
                          nauInternalCalOk ? 1 : 0,
                          static_cast<unsigned>(nauInternalCalAttempts));
            Serial0.printf("[BOOTSTATUS] vcu_mode=%s, vcu_ready=%d, rb_plus=%d, nau_ldo=%d, nau_cal=%d, nau_cal_attempts=%u\n",
                           MetaSense::Globals::kVcuSwitch ? "gpio" : "forced",
                           vcuReady ? 1 : 0,
                           rbPlusLevel,
                           nauLdoConfigured ? 1 : 0,
                           nauInternalCalOk ? 1 : 0,
                           static_cast<unsigned>(nauInternalCalAttempts));
        }

        if (now - lastStatusMs > kHeartbeatPeriodMs) {
            lastStatusMs = now;
            const wl_status_t status = WiFi.status();
            const String ip = WiFi.localIP().toString();
            const uint32_t ts = heartbeatTimestamp();
            const bool vcuReady = MetaSense::Input::isVcuReady();
            const float torqueNm = MetaSense::Input::torqueNm();
            const float egtHotC = MetaSense::Input::egtHotC();
            float ambientC = 0.0f;
            float pressureHpa = 0.0f;
            float humidityPct = 0.0f;
            float airDensity = 0.0f;
            float climateCf = 0.0f;
            MetaSense::Input::getEnvironment(ambientC,
                                             pressureHpa,
                                             humidityPct,
                                             airDensity,
                                             climateCf);
            bool nauLdoConfigured = false;
            bool nauInternalCalOk = false;
            uint8_t nauInternalCalAttempts = 0;
            MetaSense::Input::getLoadCellInitStatus(nauLdoConfigured,
                                                    nauInternalCalOk,
                                                    nauInternalCalAttempts);
            const int rbPlusLevel = digitalRead(MetaSense::Globals::kRbPlusInputPin);
            Serial.printf("[HEARTBEAT] ts=%lu, ssid=%s, wifi=%d (%s), ip=%s, ota=%s, vcu_mode=%s, vcu_ready=%d, rb_plus=%d, torque=%.2f, egt_hot=%.1f, amb=%.1f, press=%.1f, rh=%.1f, rho=%.3f, cf=%.4f, nau_ldo=%d, nau_cal=%d, nau_cal_attempts=%u\n",
                          static_cast<unsigned long>(ts),
                          wifiCredentialsConfigured() ? ssid : "<not-configured>",
                          static_cast<int>(status),
                          wifiStatusToString(status),
                          ip.c_str(),
                          otaStarted ? "ready" : "not-ready",
                          MetaSense::Globals::kVcuSwitch ? "gpio" : "forced",
                          vcuReady ? 1 : 0,
                          rbPlusLevel,
                          torqueNm,
                          egtHotC,
                          ambientC,
                          pressureHpa,
                          humidityPct,
                          airDensity,
                          climateCf,
                          nauLdoConfigured ? 1 : 0,
                          nauInternalCalOk ? 1 : 0,
                          static_cast<unsigned>(nauInternalCalAttempts));
            Serial0.printf("[HEARTBEAT] ts=%lu, ssid=%s, wifi=%d (%s), ip=%s, ota=%s, vcu_mode=%s, vcu_ready=%d, rb_plus=%d, torque=%.2f, egt_hot=%.1f, amb=%.1f, press=%.1f, rh=%.1f, rho=%.3f, cf=%.4f, nau_ldo=%d, nau_cal=%d, nau_cal_attempts=%u\n",
                           static_cast<unsigned long>(ts),
                           wifiCredentialsConfigured() ? ssid : "<not-configured>",
                           static_cast<int>(status),
                           wifiStatusToString(status),
                           ip.c_str(),
                           otaStarted ? "ready" : "not-ready",
                           MetaSense::Globals::kVcuSwitch ? "gpio" : "forced",
                           vcuReady ? 1 : 0,
                           rbPlusLevel,
                           torqueNm,
                           egtHotC,
                           ambientC,
                           pressureHpa,
                           humidityPct,
                           airDensity,
                           climateCf,
                           nauLdoConfigured ? 1 : 0,
                           nauInternalCalOk ? 1 : 0,
                           static_cast<unsigned>(nauInternalCalAttempts));

            uint32_t samplerLastUs = 0;
            uint32_t samplerMaxUs = 0;
            uint32_t samplerEmaUs = 0;
            uint32_t samplerLoops = 0;
            MetaSense::Input::getLoadCellSamplerRuntime(
                samplerLastUs,
                samplerMaxUs,
                samplerEmaUs,
                samplerLoops);

#if ENABLE_RUNTIME_INSTRUMENTATION
            Serial.printf("[RUNTIME] ctrl(us):last=%lu ema=%lu max=%lu ov=%lu loops=%lu | "
                          "net(us):last=%lu ema=%lu max=%lu ov=%lu loops=%lu | "
                          "modbus(us):last=%lu ema=%lu max=%lu ov=%lu loops=%lu | "
                          "sampler(us):last=%lu ema=%lu max=%lu loops=%lu\n",
                          static_cast<unsigned long>(controlTaskStats.lastUs),
                          static_cast<unsigned long>(controlTaskStats.emaUs),
                          static_cast<unsigned long>(controlTaskStats.maxUs),
                          static_cast<unsigned long>(controlTaskStats.overruns),
                          static_cast<unsigned long>(controlTaskStats.loops),
                          static_cast<unsigned long>(networkTaskStats.lastUs),
                          static_cast<unsigned long>(networkTaskStats.emaUs),
                          static_cast<unsigned long>(networkTaskStats.maxUs),
                          static_cast<unsigned long>(networkTaskStats.overruns),
                          static_cast<unsigned long>(networkTaskStats.loops),
                              static_cast<unsigned long>(modbusTaskStats.lastUs),
                              static_cast<unsigned long>(modbusTaskStats.emaUs),
                              static_cast<unsigned long>(modbusTaskStats.maxUs),
                              static_cast<unsigned long>(modbusTaskStats.overruns),
                              static_cast<unsigned long>(modbusTaskStats.loops),
                              static_cast<unsigned long>(samplerLastUs),
                              static_cast<unsigned long>(samplerEmaUs),
                              static_cast<unsigned long>(samplerMaxUs),
                              static_cast<unsigned long>(samplerLoops));
                Serial0.printf("[RUNTIME] ctrl(us):last=%lu ema=%lu max=%lu ov=%lu loops=%lu | "
                               "net(us):last=%lu ema=%lu max=%lu ov=%lu loops=%lu | "
                               "modbus(us):last=%lu ema=%lu max=%lu ov=%lu loops=%lu | "
                               "sampler(us):last=%lu ema=%lu max=%lu loops=%lu\n",
                               static_cast<unsigned long>(controlTaskStats.lastUs),
                               static_cast<unsigned long>(controlTaskStats.emaUs),
                               static_cast<unsigned long>(controlTaskStats.maxUs),
                               static_cast<unsigned long>(controlTaskStats.overruns),
                               static_cast<unsigned long>(controlTaskStats.loops),
                               static_cast<unsigned long>(networkTaskStats.lastUs),
                               static_cast<unsigned long>(networkTaskStats.emaUs),
                               static_cast<unsigned long>(networkTaskStats.maxUs),
                               static_cast<unsigned long>(networkTaskStats.overruns),
                               static_cast<unsigned long>(networkTaskStats.loops),
                               static_cast<unsigned long>(modbusTaskStats.lastUs),
                               static_cast<unsigned long>(modbusTaskStats.emaUs),
                               static_cast<unsigned long>(modbusTaskStats.maxUs),
                               static_cast<unsigned long>(modbusTaskStats.overruns),
                               static_cast<unsigned long>(modbusTaskStats.loops),
                               static_cast<unsigned long>(samplerLastUs),
                               static_cast<unsigned long>(samplerEmaUs),
                               static_cast<unsigned long>(samplerMaxUs),
                               static_cast<unsigned long>(samplerLoops));
                resetMaxStats();
#endif
        }

    }
}

} // anonymous namespace

void setup()
{
    Serial.begin(115200);
    Serial0.begin(115200);
    const uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < 2000U) {
        delay(10);
    }
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
    dynoVcu.begin(millis());
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
