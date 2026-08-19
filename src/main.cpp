#include <Arduino.h>
#include "controlTask.h"
#include "CommandRouter.h"
#include "CANBus.h"
#include "HardwareOutputStateMachine.h"
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

#ifndef METASENSE_LEAF_CAN_RX_ENABLED
#define METASENSE_LEAF_CAN_RX_ENABLED 1
#endif

#ifndef METASENSE_LEAF_CAN_TX_ENABLED
#define METASENSE_LEAF_CAN_TX_ENABLED 0
#endif

#ifndef METASENSE_LEAF_CAN_LISTEN_ONLY
#define METASENSE_LEAF_CAN_LISTEN_ONLY 0
#endif

#ifndef METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS
#define METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS 0
#endif

#ifndef METASENSE_LEAF_CAN_TX_PIN
#define METASENSE_LEAF_CAN_TX_PIN 4
#endif

#ifndef METASENSE_LEAF_CAN_RX_PIN
#define METASENSE_LEAF_CAN_RX_PIN 5
#endif

#ifndef METASENSE_LEAF_CAN_BITRATE_KBPS
#define METASENSE_LEAF_CAN_BITRATE_KBPS 500
#endif

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
#define METASENSE_HEARTBEAT_PERIOD_MS 2000
#endif
#ifndef METASENSE_CAN_RX_ONE_LINE_LOG
#define METASENSE_CAN_RX_ONE_LINE_LOG 0
#endif
#ifndef METASENSE_FW_ID
#define METASENSE_FW_ID "unknown"
#endif
#ifndef METASENSE_STARTUP_SNIFF_RELEASE_TX_PIN
#define METASENSE_STARTUP_SNIFF_RELEASE_TX_PIN -1
#endif
constexpr uint32_t kControlPeriodMs = METASENSE_CONTROL_PERIOD_MS;
constexpr uint32_t kModbusPeriodMs = 50;
constexpr uint32_t kHeartbeatPeriodMs = METASENSE_HEARTBEAT_PERIOD_MS;
constexpr UBaseType_t kControlTaskPriority = 5;
constexpr UBaseType_t kNetworkTaskPriority = 3;
constexpr UBaseType_t kModbusTaskPriority = 1;

void processSerialMonitorCommand(Stream& stream, String& buffer)
{
    while (stream.available() > 0) {
        const int raw = stream.read();
        if (raw < 0) {
            break;
        }

        const char ch = static_cast<char>(raw);
        if (ch == '\r') {
            continue;
        }

        if (ch != '\n') {
            if (buffer.length() < 64U) {
                buffer += ch;
            }
            continue;
        }

        buffer.trim();
        buffer.toUpperCase();
        if (buffer.isEmpty()) {
            continue;
        }

        if (buffer == "DUMP11A" || buffer == "STARTUP_SNIFF_DUMP") {
            const bool printed = MetaSense::CANBus::printStartupSniffCapture();
            if (!printed) {
                stream.println("[STARTUP-SNIFF] no completed capture available");
            }
        } else if (buffer == "HELP") {
            stream.println("[SERIAL-CMD] DUMP11A | STARTUP_SNIFF_DUMP");
        } else {
            stream.print("[SERIAL-CMD] unknown: ");
            stream.println(buffer);
        }

        buffer = "";
    }
}

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

bool setupWebServer()
{
    if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) {
        logLine("[BOOT] LittleFS mount failed (no auto-format)");
        logLine("[BOOT] Preserve FS contents; run Upload Filesystem manually if needed");
        return false;
    }

    auto sendHtmlNoCache = [](AsyncWebServerRequest* request, const char* path) {
        if (!LittleFS.exists(path)) {
            const String body = String("<html><body><h1>MetaSense DYNO</h1><p>UI file missing: ") +
                                path +
                                "</p><p>LittleFS was likely reformatted. Upload the filesystem image to restore the web UI.</p>"
                                "</body></html>";
            AsyncWebServerResponse* fallback = request->beginResponse(503, "text/html", body);
            fallback->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
            fallback->addHeader("Pragma", "no-cache");
            fallback->addHeader("Expires", "0");
            request->send(fallback);
            return;
        }

        AsyncWebServerResponse* response = request->beginResponse(LittleFS, path, "text/html");
        if (response == nullptr) {
            request->send(500, "text/plain", "failed to create file response");
            return;
        }
        response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
    };

    webServer.on("/", HTTP_GET, [sendHtmlNoCache](AsyncWebServerRequest* request) {
        sendHtmlNoCache(request, "/index.html");
    });
    webServer.on("/index.html", HTTP_GET, [sendHtmlNoCache](AsyncWebServerRequest* request) {
        sendHtmlNoCache(request, "/index.html");
    });
    webServer.on("/index1.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/index.html");
    });
    webServer.on("/index1", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/index.html");
    });
    webServer.on("/index", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/index.html");
    });
    webServer.on("/settings", HTTP_GET, [sendHtmlNoCache](AsyncWebServerRequest* request) {
        sendHtmlNoCache(request, "/settings.html");
    });
    webServer.on("/trend", HTTP_GET, [sendHtmlNoCache](AsyncWebServerRequest* request) {
        sendHtmlNoCache(request, "/trend.html");
    });
    webServer.on("/update", HTTP_GET, [sendHtmlNoCache](AsyncWebServerRequest* request) {
        sendHtmlNoCache(request, "/update.html");
    });
    webServer.on("/update_fs", HTTP_GET, [sendHtmlNoCache](AsyncWebServerRequest* request) {
        sendHtmlNoCache(request, "/update_fs.html");
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

    webServer.serveStatic("/", LittleFS, "/");
    MetaSense::WebSocketServer::begin(webServer);
    webServer.begin();
    logLine("[BOOT] Web server ready on port 80");
    if (!LittleFS.exists("/index.html")) {
        logLine("[BOOT] Web UI assets missing from LittleFS; upload filesystem image to restore UI pages");
    }
    return true;
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

void logBuildProfile()
{
    Serial.printf("[BOOT] Build profile: vcu_gpio=%d, vcu_sim=%d, hv_vcu_owner=%d\n",
                  VCU_switch != 0 ? 1 : 0,
                  METASENSE_VCU_SIM_MODE != 0 ? 1 : 0,
                  METASENSE_VCU_OWNS_HV_RPLUS_PRECHARGE != 0 ? 1 : 0);
    Serial0.printf("[BOOT] Build profile: vcu_gpio=%d, vcu_sim=%d, hv_vcu_owner=%d\n",
                   VCU_switch != 0 ? 1 : 0,
                   METASENSE_VCU_SIM_MODE != 0 ? 1 : 0,
                   METASENSE_VCU_OWNS_HV_RPLUS_PRECHARGE != 0 ? 1 : 0);

    Serial.printf("[BOOT] Build profile: leaf_can_rx=%d, leaf_can_tx=%d, can_listen_only=%d, can_bitrate_kbps=%d, can_tx_pin=%d, can_rx_pin=%d, leaf_sim_feedback=%d\n",
                  METASENSE_LEAF_CAN_RX_ENABLED != 0 ? 1 : 0,
                  METASENSE_LEAF_CAN_TX_ENABLED != 0 ? 1 : 0,
                  METASENSE_LEAF_CAN_LISTEN_ONLY != 0 ? 1 : 0,
                  METASENSE_LEAF_CAN_BITRATE_KBPS,
                  METASENSE_LEAF_CAN_TX_PIN,
                  METASENSE_LEAF_CAN_RX_PIN,
                  METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS != 0 ? 1 : 0);
    Serial0.printf("[BOOT] Build profile: leaf_can_rx=%d, leaf_can_tx=%d, can_listen_only=%d, can_bitrate_kbps=%d, can_tx_pin=%d, can_rx_pin=%d, leaf_sim_feedback=%d\n",
                   METASENSE_LEAF_CAN_RX_ENABLED != 0 ? 1 : 0,
                   METASENSE_LEAF_CAN_TX_ENABLED != 0 ? 1 : 0,
                   METASENSE_LEAF_CAN_LISTEN_ONLY != 0 ? 1 : 0,
                   METASENSE_LEAF_CAN_BITRATE_KBPS,
                   METASENSE_LEAF_CAN_TX_PIN,
                   METASENSE_LEAF_CAN_RX_PIN,
                   METASENSE_LEAF_SIM_FEEDBACK_WITHOUT_BUS != 0 ? 1 : 0);
    Serial.printf("[FW-ID] id=%s hb_ms=%lu\n", METASENSE_FW_ID, static_cast<unsigned long>(kHeartbeatPeriodMs));
    Serial0.printf("[FW-ID] id=%s hb_ms=%lu\n", METASENSE_FW_ID, static_cast<unsigned long>(kHeartbeatPeriodMs));
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
    Serial.println("[WiFi] DHCP mode");
    Serial0.println("[WiFi] DHCP mode");

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

    logLine("[BOOT] WiFi config ready; connection will be attempted by network task");
}

void setupOtaOnceConnected()
{
    static uint32_t lastWebServerAttemptMs = 0U;

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
        const uint32_t nowMs = millis();
        if (lastWebServerAttemptMs == 0U || (nowMs - lastWebServerAttemptMs) >= 5000U) {
            lastWebServerAttemptMs = nowMs;
            webServerStarted = setupWebServer();
        }
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
                    dynoVcu.getRbPlus(),
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
                         dynoVcu.getRbPlus(),
                         dynoVcu.getPrecharge(),
                         dynoVcu.getSSR(),
                         dynoVcu.getRMinus());

    // The hardware output state machine owns the relay state. Keep the VCU relay
    // override disabled so the HWSM remains the single enforcement point for the
    // RSS/RB-minus interlock.
    MetaSense::HardwareOutputStateMachine::setVcuRelayOverride(false, false, false, false, false);
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
    uint32_t lastWifiRetryMs = 0;
    wl_status_t lastWifiStatus = WL_IDLE_STATUS;
    String serialCommandBuffer;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25));
        const uint32_t startedUs = micros();

        processSerialMonitorCommand(Serial, serialCommandBuffer);

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
        const wl_status_t wifiStatusNow = WiFi.status();

        if (wifiStatusNow != lastWifiStatus) {
            Serial.printf("[WiFi] Status transition: %d (%s) -> %d (%s)\n",
                          static_cast<int>(lastWifiStatus),
                          wifiStatusToString(lastWifiStatus),
                          static_cast<int>(wifiStatusNow),
                          wifiStatusToString(wifiStatusNow));
            Serial0.printf("[WiFi] Status transition: %d (%s) -> %d (%s)\n",
                           static_cast<int>(lastWifiStatus),
                           wifiStatusToString(lastWifiStatus),
                           static_cast<int>(wifiStatusNow),
                           wifiStatusToString(wifiStatusNow));
            lastWifiStatus = wifiStatusNow;
        }

        if (wifiCredentialsConfigured() &&
            wifiStatusNow != WL_CONNECTED &&
            (lastWifiRetryMs == 0U || (now - lastWifiRetryMs) >= 10000U)) {
            lastWifiRetryMs = now;
            Serial.printf("[WiFi] Retry connect (status=%d %s)\n",
                          static_cast<int>(wifiStatusNow),
                          wifiStatusToString(wifiStatusNow));
            Serial0.printf("[WiFi] Retry connect (status=%d %s)\n",
                           static_cast<int>(wifiStatusNow),
                           wifiStatusToString(wifiStatusNow));
            // Set static IP before connecting
            IPAddress ip(192, 168, 0, 211);
            IPAddress gateway(192, 168, 0, 1);
            IPAddress subnet(255, 255, 255, 0);
            IPAddress dns(8, 8, 8, 8);
            WiFi.config(ip, gateway, subnet, dns);
            WiFi.begin(ssid, password);
        }

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
            const bool ssrActive = MetaSense::HardwareOutputStateMachine::isSsrActive();
            Serial.printf("[BOOTSTATUS] vcu_mode=%s, vcu_ready=%d, rb_plus=%d, ssr=%d, nau_ldo=%d, nau_cal=%d, nau_cal_attempts=%u\n",
                          "can_inverter_status",
                          vcuReady ? 1 : 0,
                          rbPlusLevel,
                          ssrActive ? 1 : 0,
                          nauLdoConfigured ? 1 : 0,
                          nauInternalCalOk ? 1 : 0,
                          static_cast<unsigned>(nauInternalCalAttempts));
            Serial0.printf("[BOOTSTATUS] vcu_mode=%s, vcu_ready=%d, rb_plus=%d, ssr=%d, nau_ldo=%d, nau_cal=%d, nau_cal_attempts=%u\n",
                           "can_inverter_status",
                           vcuReady ? 1 : 0,
                           rbPlusLevel,
                           ssrActive ? 1 : 0,
                           nauLdoConfigured ? 1 : 0,
                           nauInternalCalOk ? 1 : 0,
                           static_cast<unsigned>(nauInternalCalAttempts));
        }

        if (now - lastStatusMs > kHeartbeatPeriodMs) {
            lastStatusMs = now;
            const wl_status_t status = WiFi.status();
            const String ip = WiFi.localIP().toString();
            const uint32_t ts = heartbeatTimestamp();
            const LeafInvFeedback& leafFbDiag = MetaSense::CANBus::feedback();
            const MetaSense::CANBus::Stats& canStatsDiag = MetaSense::CANBus::stats();
            const MetaSense::CANBus::StartupSniffStatus startupSniffDiag = MetaSense::CANBus::startupSniffStatus();
            const bool vcuReady = MetaSense::Input::isVcuReady();
            const char* vcuReadySource = MetaSense::Input::vcuReadySource();
            const bool hwPrestartWarn = MetaSense::HardwareOutputStateMachine::hasPrestartWarning();
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
            const bool ssrActive = MetaSense::HardwareOutputStateMachine::isSsrActive();
            Serial.printf("[HEARTBEAT] ts=%lu, ssid=%s, wifi=%d (%s), ip=%s, ota=%s, vcu_mode=%s, vcu_ready=%d, vcu_ready_src=%s, prestart_warn=%d, rb_plus=%d, ssr=%d, torque=%.2f, egt_hot=%.1f, amb=%.1f, press=%.1f, rh=%.1f, rho=%.3f, cf=%.4f, nau_ldo=%d, nau_cal=%d, nau_cal_attempts=%u, can_cfg_rx=%d, can_cfg_oneline=%d, sniff_en=%d, sniff_active=%d, sniff_done=%d, sniff_dumped=%d, sniff_count=%u, sniff_drop=%u, can_rx_total=%lu, can_last_id=0x%03lX, can_11a=%lu, can_50b=%lu, rpm_raw01_le=%u, rpm_raw01_be=%u, rpm_raw23_le=%u, rpm_raw23_be=%u, tq_raw01_le=%d, tq_raw01_be=%d, tq_raw23_le=%d, tq_raw23_be=%d\n",
                          static_cast<unsigned long>(ts),
                          wifiCredentialsConfigured() ? ssid : "<not-configured>",
                          static_cast<int>(status),
                          wifiStatusToString(status),
                          ip.c_str(),
                          otaStarted ? "ready" : "not-ready",
                          "can_inverter_status",
                          vcuReady ? 1 : 0,
                          vcuReadySource,
                          hwPrestartWarn ? 1 : 0,
                          rbPlusLevel,
                          ssrActive ? 1 : 0,
                          torqueNm,
                          egtHotC,
                          ambientC,
                          pressureHpa,
                          humidityPct,
                          airDensity,
                          climateCf,
                          nauLdoConfigured ? 1 : 0,
                          nauInternalCalOk ? 1 : 0,
                          static_cast<unsigned>(nauInternalCalAttempts),
                          METASENSE_LEAF_CAN_RX_ENABLED != 0 ? 1 : 0,
                          METASENSE_CAN_RX_ONE_LINE_LOG != 0 ? 1 : 0,
                          startupSniffDiag.enabled ? 1 : 0,
                          startupSniffDiag.active ? 1 : 0,
                          startupSniffDiag.done ? 1 : 0,
                          startupSniffDiag.dumped ? 1 : 0,
                          static_cast<unsigned>(startupSniffDiag.count),
                          static_cast<unsigned>(startupSniffDiag.dropped),
                          static_cast<unsigned long>(canStatsDiag.rxFrames),
                          static_cast<unsigned long>(canStatsDiag.lastRxId),
                          static_cast<unsigned long>(canStatsDiag.rx11aFrames),
                          static_cast<unsigned long>(canStatsDiag.rx50bFrames),
                          static_cast<unsigned>(leafFbDiag.rpm_raw01_le),
                          static_cast<unsigned>(leafFbDiag.rpm_raw01_be),
                          static_cast<unsigned>(leafFbDiag.rpm_raw23_le),
                          static_cast<unsigned>(leafFbDiag.rpm_raw23_be),
                          static_cast<int>(leafFbDiag.torque_raw01_le),
                          static_cast<int>(leafFbDiag.torque_raw01_be),
                          static_cast<int>(leafFbDiag.torque_raw23_le),
                          static_cast<int>(leafFbDiag.torque_raw23_be));
            Serial0.printf("[HEARTBEAT] ts=%lu, ssid=%s, wifi=%d (%s), ip=%s, ota=%s, vcu_mode=%s, vcu_ready=%d, vcu_ready_src=%s, prestart_warn=%d, rb_plus=%d, ssr=%d, torque=%.2f, egt_hot=%.1f, amb=%.1f, press=%.1f, rh=%.1f, rho=%.3f, cf=%.4f, nau_ldo=%d, nau_cal=%d, nau_cal_attempts=%u, can_cfg_rx=%d, can_cfg_oneline=%d, sniff_en=%d, sniff_active=%d, sniff_done=%d, sniff_dumped=%d, sniff_count=%u, sniff_drop=%u, can_rx_total=%lu, can_last_id=0x%03lX, can_11a=%lu, can_50b=%lu, rpm_raw01_le=%u, rpm_raw01_be=%u, rpm_raw23_le=%u, rpm_raw23_be=%u, tq_raw01_le=%d, tq_raw01_be=%d, tq_raw23_le=%d, tq_raw23_be=%d\n",
                           static_cast<unsigned long>(ts),
                           wifiCredentialsConfigured() ? ssid : "<not-configured>",
                           static_cast<int>(status),
                           wifiStatusToString(status),
                           ip.c_str(),
                           otaStarted ? "ready" : "not-ready",
                           "can_inverter_status",
                           vcuReady ? 1 : 0,
                           vcuReadySource,
                           hwPrestartWarn ? 1 : 0,
                           rbPlusLevel,
                           ssrActive ? 1 : 0,
                           torqueNm,
                           egtHotC,
                           ambientC,
                           pressureHpa,
                           humidityPct,
                           airDensity,
                           climateCf,
                           nauLdoConfigured ? 1 : 0,
                           nauInternalCalOk ? 1 : 0,
                           static_cast<unsigned>(nauInternalCalAttempts),
                           METASENSE_LEAF_CAN_RX_ENABLED != 0 ? 1 : 0,
                           METASENSE_CAN_RX_ONE_LINE_LOG != 0 ? 1 : 0,
                           startupSniffDiag.enabled ? 1 : 0,
                           startupSniffDiag.active ? 1 : 0,
                           startupSniffDiag.done ? 1 : 0,
                           startupSniffDiag.dumped ? 1 : 0,
                           static_cast<unsigned>(startupSniffDiag.count),
                           static_cast<unsigned>(startupSniffDiag.dropped),
                           static_cast<unsigned long>(canStatsDiag.rxFrames),
                           static_cast<unsigned long>(canStatsDiag.lastRxId),
                           static_cast<unsigned long>(canStatsDiag.rx11aFrames),
                           static_cast<unsigned long>(canStatsDiag.rx50bFrames),
                           static_cast<unsigned>(leafFbDiag.rpm_raw01_le),
                           static_cast<unsigned>(leafFbDiag.rpm_raw01_be),
                           static_cast<unsigned>(leafFbDiag.rpm_raw23_le),
                           static_cast<unsigned>(leafFbDiag.rpm_raw23_be),
                           static_cast<int>(leafFbDiag.torque_raw01_le),
                           static_cast<int>(leafFbDiag.torque_raw01_be),
                           static_cast<int>(leafFbDiag.torque_raw23_le),
                           static_cast<int>(leafFbDiag.torque_raw23_be));

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
    delay(100);  // Let Serial stabilize
    // Force flush and test immediate output
    Serial.write("[TEST] Serial online\n");
    Serial.flush();
    Serial0.write("[TEST] Serial0 online\n");
    Serial0.flush();
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
    logBuildProfile();
    if (METASENSE_STARTUP_SNIFF_RELEASE_TX_PIN >= 0) {
        pinMode(METASENSE_STARTUP_SNIFF_RELEASE_TX_PIN, INPUT_PULLUP);
        Serial.printf("[BOOT] Released legacy CAN TX pin %d to INPUT_PULLUP\n",
                      METASENSE_STARTUP_SNIFF_RELEASE_TX_PIN);
        Serial0.printf("[BOOT] Released legacy CAN TX pin %d to INPUT_PULLUP\n",
                       METASENSE_STARTUP_SNIFF_RELEASE_TX_PIN);
    }
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
