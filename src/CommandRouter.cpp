#include <Arduino.h>
#include <ArduinoJson.h>
#include <sys/time.h>
#include <Preferences.h>

#include <ESPAsyncWebServer.h>
#include "DynoStateMachine.h"
#include "Settings.h"
#include "Input.h"
class AsyncWebSocketClient;

namespace MetaSense::RunStorage {
void saveCalibration();
bool saveRun(const String& payload);
String listRuns();
String loadRunPoints(const String& filename);
bool deleteRunByIndex(int index);
}

namespace MetaSense::WebSocketServer {
AsyncWebSocket& socket();
void sendStatus(const String& msg);
void sendInfo(const String& msg);
}

namespace MetaSense::Input {
void tare();
void setCalibrationFactor(float factor);
}

namespace {

float calibrationZero = 0.0f;
float calibrationFactor = 1.0f;
String currentCustomer;
String currentUnit;
String currentComments;

bool parseCalibrationFactorCommand(const String& cmd, float& factor)
{
    const String prefix = "CALIBRATE_FACTOR:";
    if (!cmd.startsWith(prefix)) {
        return false;
    }

    const String value = cmd.substring(prefix.length());
    if (value.length() == 0) {
        return false;
    }

    factor = value.toFloat();
    return true;
}

void sendSettingsSnapshot()
{
    String json;
    json.reserve(512);

    json = "{\"type\":\"settings\"";
    json += ",\"calFactor\":" + String(calibrationFactor, 6);
    json += ",\"maxRPM\":" + String(MetaSense::Settings::maxRPM, 0);
    json += ",\"maxHP\":" + String(MetaSense::Settings::maxHP, 1);
    json += ",\"maxTorque\":" + String(MetaSense::Settings::maxTorque, 1);
    json += ",\"armCm\":" + String(MetaSense::Settings::armCm, 1);
    json += ",\"pulsesPerRev\":" + String(MetaSense::Settings::pulsesPerRev, 2);
    json += ",\"rpmFilter\":" + String(MetaSense::Settings::filterAlpha, 3);
    json += ",\"mode\":" + String(MetaSense::Settings::inertiaMode ? "\"inertia\"" : "\"brake\"");
    json += ",\"virtGearRatio\":" + String(MetaSense::Settings::virtGearRatio, 3);
    json += ",\"drumMass\":" + String(MetaSense::Settings::drumMassKg, 2);
    json += ",\"drumCm\":" + String(MetaSense::Settings::drumRadiusM * 200.0f, 1);
    json += ",\"pulsesPerRevDrum\":" + String(MetaSense::Settings::pulsesPerRevDrum, 2);
    json += ",\"drumInertiaType\":\"" + String(MetaSense::Settings::drumWallM > 0.0f ? "hollow" : "solid") + "\"";
    json += ",\"drumWallCm\":" + String(MetaSense::Settings::drumWallM * 100.0f, 1);
    json += ",\"drumInertiaCustom\":" + String(MetaSense::Settings::drumInertiaKgM2, 4);
    json += ",\"drumInertiaKgM2\":" + String(MetaSense::Settings::drumInertiaKgM2, 4);
    json += ",\"brakeToEngineRatio\":" + String(MetaSense::Settings::virtGearRatio, 3);
    json += ",\"drivetrainEff\":" + String(MetaSense::Settings::drivetrainEff, 1);
    json += ",\"rpmSource\":\"" + String(MetaSense::Settings::useCanLeafRpm ? "leafrpm" : "tachogen") + "\"";
    json += ",\"version\":\"3.0\"";
    json += ",\"buildDate\":\"" __DATE__ " " __TIME__ "\"";
    json += "}";

    MetaSense::WebSocketServer::socket().textAll(json);
}

} // anonymous namespace

namespace MetaSense::WebSocketServer {

void sendStatus(const String& msg)
{
    String json;
    json.reserve(96);
    json = "{\"type\":\"status\",\"msg\":\"" + msg + "\"}";
    socket().textAll(json);
}

void sendInfo(const String& msg)
{
    String json;
    json.reserve(128);
    json = "{\"type\":\"info\",\"msg\":\"" + msg + "\"}";
    socket().textAll(json);
}

} // namespace MetaSense::WebSocketServer

namespace MetaSense::Input {

void __attribute__((weak)) tare()
{
}

void __attribute__((weak)) setCalibrationFactor(float)
{
}

} // namespace MetaSense::Input

namespace MetaSense::RunStorage {

void __attribute__((weak)) saveCalibration()
{
}

} // namespace MetaSense::RunStorage
namespace MetaSense::CommandRouter {

void handleWebSocketMessage(AsyncWebSocketClient *client, const String& msg)
{
    (void)client;

    String cmd = "";
    float value = 0.0f;

    #pragma GCC diagnostic push
#   pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    StaticJsonDocument<256> doc;
    #pragma GCC diagnostic pop

    if (!deserializeJson(doc, msg)) {
        cmd = String((const char*)(doc["cmd"] | ""));
        value = doc["value"] | 0.0f;
    } else {
        cmd = msg;
        cmd.trim();
    }

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();
    const bool isAutoRunCommand = cmdUpper.startsWith("AUTO_RUN:") || cmdUpper.startsWith("AUTU_RUN:");

    if (cmdUpper == "START") {
        MetaSense::DynoStateMachine::startRecording();
        MetaSense::WebSocketServer::sendStatus("Recording started");
    }
    else if (cmdUpper == "STOP") {
        MetaSense::DynoStateMachine::stopRecording();
        MetaSense::WebSocketServer::sendStatus("Recording stopped");
    }
    else if (cmdUpper == "SET_RPM") {
        MetaSense::Settings::setRpmTarget(value);
        MetaSense::WebSocketServer::sendStatus("RPM target set");
    }
    else if (cmdUpper == "RPM_DIAG") {
        MetaSense::WebSocketServer::sendInfo("RPM diagnostic requested");
    }
    else if (cmdUpper == "SET_TACHO_CAL") {
        float cal = value;
        if (cal == 0.0f && !doc.isNull()) {
            cal = 10.0f;
        }
        MetaSense::Settings::setTachoCal(cal);
        MetaSense::WebSocketServer::sendStatus("Tachogen calibration updated");
    }
    else if (cmdUpper == "SET_TORQUE_FF") {
        MetaSense::DynoStateMachine::setTorqueFeedForward(value);
        MetaSense::WebSocketServer::sendStatus("Torque feed-forward updated");
    }
    else if (cmdUpper == "TARE" || cmdUpper == "CALIBRATE_ZERO") {
        MetaSense::Input::tare();
        calibrationZero = 0.0f;
        MetaSense::WebSocketServer::sendStatus("Tare applied");
    }
    else if (isAutoRunCommand) {
        const int firstColon = cmd.indexOf(':');
        const int secondColon = cmd.indexOf(':', firstColon + 1);
        if (firstColon < 0 || secondColon < 0) {
            MetaSense::WebSocketServer::sendInfo("Invalid AUTO_RUN format");
            return;
        }

        const float startRpm = cmd.substring(firstColon + 1, secondColon).toFloat();
        const float endRpm = cmd.substring(secondColon + 1).toFloat();
        if (startRpm <= 0.0f || endRpm <= startRpm) {
            MetaSense::WebSocketServer::sendInfo("Invalid AUTO_RUN range");
            return;
        }

        MetaSense::Settings::setRpmStart(startRpm);
        MetaSense::Settings::setRpmEnd(endRpm);
        MetaSense::DynoStateMachine::setPanelAuto(true);
        MetaSense::DynoStateMachine::setAutoMode(true);
        MetaSense::DynoStateMachine::startRecording();
        MetaSense::WebSocketServer::sendStatus("Auto run armed");
    }
    else if (cmdUpper == "MANUAL_START") {
        MetaSense::DynoStateMachine::setAutoMode(false);
        MetaSense::DynoStateMachine::setPanelAuto(false);
        MetaSense::DynoStateMachine::startRecording();
        MetaSense::WebSocketServer::sendStatus("Manual recording started");
    }
    else if (cmdUpper == "MANUAL_STOP") {
        const bool autoWasActive = MetaSense::DynoStateMachine::isAutoRunActive();
        if (autoWasActive) {
            MetaSense::DynoStateMachine::setAutoMode(false);
            MetaSense::DynoStateMachine::setPanelAuto(false);
        }
        MetaSense::DynoStateMachine::stopRecording();
        MetaSense::WebSocketServer::sendStatus(autoWasActive ? "Auto run stopped" : "Manual recording stopped");

        // GUI waits for this event to finalize report modal after manual stop.
        MetaSense::WebSocketServer::socket().textAll(
            "{\"type\":\"run_complete\",\"peakKW\":0,\"peakKW_RPM\":0,\"peakTorque\":0,\"peakTorque_RPM\":0,\"peakEGT\":0}");
    }
    else if (cmdUpper == "CANCEL_AUTO_RUN") {
        MetaSense::DynoStateMachine::setAutoMode(false);
        MetaSense::DynoStateMachine::setPanelAuto(false);
        MetaSense::DynoStateMachine::abortAutoRun();
        MetaSense::WebSocketServer::sendStatus("Auto run cancelled");

        // Mirror manual-stop behavior so GUI can finalize and enable reporting.
        MetaSense::WebSocketServer::socket().textAll(
            "{\"type\":\"run_complete\",\"peakKW\":0,\"peakKW_RPM\":0,\"peakTorque\":0,\"peakTorque_RPM\":0,\"peakEGT\":0}");
    }
    else if (cmdUpper.startsWith("SET_CUSTOMER:") || cmdUpper.startsWith("SET CUSTOMER:")) {
        const int sep = cmd.indexOf(':');
        if (sep < 0) {
            MetaSense::WebSocketServer::sendInfo("Invalid customer payload");
            return;
        }

        const String payload = cmd.substring(sep + 1);
        const int p1 = payload.indexOf('|');
        const int p2 = (p1 >= 0) ? payload.indexOf('|', p1 + 1) : -1;

        if (p1 < 0) {
            currentCustomer = payload;
            currentUnit = "";
            currentComments = "";
        } else {
            currentCustomer = payload.substring(0, p1);
            if (p2 < 0) {
                currentUnit = payload.substring(p1 + 1);
                currentComments = "";
            } else {
                currentUnit = payload.substring(p1 + 1, p2);
                currentComments = payload.substring(p2 + 1);
            }
        }

        // Metadata-only command used by GUI before run start.
    }
    else if (cmd.startsWith("CALIBRATE_FACTOR:")) {
        float factor = 0.0f;
        if (!parseCalibrationFactorCommand(cmd, factor)) {
            MetaSense::WebSocketServer::sendInfo("Invalid calibration factor");
            return;
        }

        MetaSense::Input::setCalibrationFactor(factor);
        calibrationFactor = factor;
        MetaSense::RunStorage::saveCalibration();
        MetaSense::WebSocketServer::sendStatus("Calibration factor updated");
    }
    else if (cmdUpper == "GET_CALIBRATION") {
        String info;
        info.reserve(64);
        info = "zero=" + String(calibrationZero, 6) + ",factor=" + String(calibrationFactor, 6);
        MetaSense::WebSocketServer::sendInfo(info);
    }
    else if (cmdUpper.startsWith("WIFI_CLIENT:")) {
        const String payload = cmd.substring(12);
        const int sep = payload.indexOf(':');
        if (sep <= 0) {
            MetaSense::WebSocketServer::sendInfo("Invalid WIFI_CLIENT payload");
            return;
        }

        const String ssid = payload.substring(0, sep);
        const String pass = payload.substring(sep + 1);

        Preferences prefs;
        if (!prefs.begin("wifi", false)) {
            MetaSense::WebSocketServer::sendInfo("Failed to open WiFi storage");
            return;
        }

        prefs.putString("ssid", ssid);
        prefs.putString("pass", pass);
        prefs.end();

        MetaSense::WebSocketServer::sendStatus("WiFi credentials saved");
        MetaSense::WebSocketServer::sendInfo("Rebooting to apply WiFi settings");
        delay(250);
        ESP.restart();
    }
    else if (cmdUpper == "GET_SETTINGS" || cmdUpper == "GET.SETTINGS") {
        sendSettingsSnapshot();
    }
    else if (cmdUpper == "PAGE_MAIN" || cmdUpper == "PAGE_SETTINGS") {
        // Presence/heartbeat commands from browser pages.
    }
    else if (cmdUpper.startsWith("SET_TIME:")) {
        // Browser sends its Unix epoch so the ESP32 gets a real wall-clock time
        // even on a local-only network where NTP is unavailable.
        const String tsStr = cmd.substring(9);
        const uint32_t epoch = (uint32_t)tsStr.toInt();
        if (epoch > 1000000000UL) {
            timeval tv { (time_t)epoch, 0 };
            settimeofday(&tv, nullptr);
        }
    }
    else if (cmdUpper == "GET_RUNS") {
        String list = MetaSense::RunStorage::listRuns();
        MetaSense::WebSocketServer::socket().textAll("{\"type\":\"runs\",\"data\":" + list + "}");
    }
    else if (cmdUpper.startsWith("GET_RUN_DATA:")) {
        String filename = cmd.substring(13);
        filename.trim();
        String points = MetaSense::RunStorage::loadRunPoints(filename);
        MetaSense::WebSocketServer::socket().textAll(
            "{\"type\":\"run_data\",\"filename\":\"" + filename + "\",\"points\":" + points + "}");
    }
    else if (cmdUpper.startsWith("DELETE_RUN:")) {
        int idx = cmd.substring(11).toInt();
        if (MetaSense::RunStorage::deleteRunByIndex(idx)) {
            MetaSense::WebSocketServer::sendInfo("Run deleted");
        } else {
            MetaSense::WebSocketServer::sendInfo("Delete failed: index out of range");
        }
    }
    else if (cmdUpper.startsWith("SAVE_RUN_DATA:")) {
        const String payload = cmd.substring(14);
        if (MetaSense::RunStorage::saveRun(payload)) {
            MetaSense::WebSocketServer::sendStatus("Run saved");
        } else {
            MetaSense::WebSocketServer::sendInfo("Run save failed");
        }
    }
    else if (cmdUpper == "RESET_ALL_SETTINGS") {
        MetaSense::Settings::setInertiaMode(false);
        MetaSense::Settings::setDrumParams(10.0f, 0.15f, 0.0f);
        MetaSense::Settings::virtGearRatio = 1.0f;
        sendSettingsSnapshot();
        MetaSense::WebSocketServer::sendStatus("Settings reset to defaults");
    }
    else if (cmdUpper.startsWith("SET_SINGLE:")) {
        const String kv = cmd.substring(11);
        const int eq = kv.indexOf('=');
        if (eq <= 0) {
            MetaSense::WebSocketServer::sendInfo("Invalid SET_SINGLE format");
            return;
        }
        const String key = kv.substring(0, eq);
        const String val = kv.substring(eq + 1);
        const float fval = val.toFloat();

        if (key == "rpmFilter") {
            MetaSense::Settings::filterAlpha = fval;
        } else if (key == "maxRPM") {
            MetaSense::Settings::maxRPM = (fval > 0.0f) ? fval : 18000.0f;
        } else if (key == "maxHP") {
            MetaSense::Settings::maxHP = (fval > 0.0f) ? fval : 25.0f;
        } else if (key == "maxTorque") {
            MetaSense::Settings::maxTorque = (fval > 0.0f) ? fval : 200.0f;
        } else if (key == "armCm") {
            MetaSense::Settings::armCm = (fval > 0.0f) ? fval : 20.0f;
        } else if (key == "pulsesPerRev") {
            MetaSense::Settings::pulsesPerRev = (fval > 0.0f) ? fval : 1.0f;
        } else if (key == "pulsesPerRevDrum") {
            MetaSense::Settings::pulsesPerRevDrum = (fval > 0.0f) ? fval : 1.0f;
        } else if (key == "drivetrainEff") {
            MetaSense::Settings::drivetrainEff = (fval > 0.0f && fval <= 100.0f) ? fval : 95.0f;
        } else if (key == "rpmSource") {
            String src = val;
            src.toLowerCase();
            // Backward compatibility with existing UI aliases.
            if (src == "leafrpm" || src == "can" || src == "spark") {
                MetaSense::Settings::useCanLeafRpm = true;
            } else if (src == "tachogen" || src == "tacho" || src == "brake" || src == "drum" || src == "cvt") {
                MetaSense::Settings::useCanLeafRpm = false;
            }
        } else if (key == "brakeToEngineRatio") {
            MetaSense::Settings::virtGearRatio = (fval > 0.0f) ? fval : 1.0f;
        } else if (key == "virtGearRatio") {
            MetaSense::Settings::virtGearRatio = (fval > 0.0f) ? fval : 1.0f;
        } else if (key == "dynoMode") {
            MetaSense::Settings::setInertiaMode(val == "inertia");
        } else if (key == "drumMass") {
            MetaSense::Settings::setDrumParams(
                fval,
                MetaSense::Settings::drumRadiusM,
                MetaSense::Settings::drumWallM);
        } else if (key == "drumCm") {
            // settings page sends diameter in cm, we store radius in m
            MetaSense::Settings::setDrumParams(
                MetaSense::Settings::drumMassKg,
                (fval / 2.0f) / 100.0f,
                MetaSense::Settings::drumWallM);
        } else if (key == "drumWallCm") {
            MetaSense::Settings::setDrumParams(
                MetaSense::Settings::drumMassKg,
                MetaSense::Settings::drumRadiusM,
                fval / 100.0f);
        } else if (key == "drumInertiaType") {
            // "custom" means drumInertiaCustom field takes precedence; handled when it arrives
            if (val != "custom") {
                // recompute from mass/radius/wall by re-applying current params
                MetaSense::Settings::setDrumParams(
                    MetaSense::Settings::drumMassKg,
                    MetaSense::Settings::drumRadiusM,
                    MetaSense::Settings::drumWallM);
            }
        } else if (key == "drumInertiaCustom") {
            MetaSense::Settings::setDrumInertiaCustom(fval);
        } else {
            // Pass-through for display-only keys (maxRPM, maxHP, armCm, etc.)
        }
        sendSettingsSnapshot();
        MetaSense::WebSocketServer::sendStatus(key + " saved");
    }
    else {
        MetaSense::WebSocketServer::sendInfo("Unknown command: " + cmd);
    }
}

} // namespace MetaSense::CommandRouter
