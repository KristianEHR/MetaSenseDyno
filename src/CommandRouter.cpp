#include <Arduino.h>
#include <ArduinoJson.h>
#include <sys/time.h>
#include <Preferences.h>
#include <LittleFS.h>

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
void tareMainGui();
void tare();
void setCalibrationFactor(float factor);
bool calibrateWithKnownWeight(float knownWeightKg, float& outFactor);
float getCalibrationFactor();
float getZeroOffset();
bool isVcuReady();
}

namespace MetaSense::ControlTask {
void configurePI(float kp, float ki, float outMin, float outMax);
}

namespace {

float calibrationZero = 0.0f;
float calibrationFactor = 0.01f;
String currentCustomer;
String currentUnit;
String currentComments;
constexpr const char* kFactoryProfilePath = "/factory_profile.json";

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

bool parseKnownWeightCommand(const String& cmd, float& knownWeightKg)
{
    const int sep = cmd.indexOf(':');
    if (sep < 0 || sep + 1 >= static_cast<int>(cmd.length())) {
        return false;
    }

    const String value = cmd.substring(sep + 1);
    knownWeightKg = value.toFloat();
    return knownWeightKg > 0.0f;
}

String buildProfilePayload(bool includeTypeEnvelope)
{
    calibrationFactor = MetaSense::Input::getCalibrationFactor();
    calibrationZero = MetaSense::Input::getZeroOffset();

    String json;
    json.reserve(900);

    if (includeTypeEnvelope) {
        json = "{\"type\":\"profile\",\"profile\":{";
    } else {
        json = "{\"profile\":{";
    }
    json += "\"schema\":1";
    json += ",\"calFactor\":" + String(calibrationFactor, 6);
    json += ",\"calZero\":" + String(calibrationZero, 6);
    json += ",\"maxRPM\":" + String(MetaSense::Settings::maxRPM, 0);
    json += ",\"maxHP\":" + String(MetaSense::Settings::maxHP, 1);
    json += ",\"maxTorque\":" + String(MetaSense::Settings::maxTorque, 1);
    json += ",\"armCm\":" + String(MetaSense::Settings::armCm, 1);
    json += ",\"kp\":" + String(MetaSense::Settings::kp, 4);
    json += ",\"ki\":" + String(MetaSense::Settings::ki, 4);
    json += ",\"kpSource\":\"" + String(MetaSense::Settings::usePot3Kp ? "pot3" : "firmware") + "\"";
    json += ",\"rhOffsetPct\":" + String(MetaSense::Settings::ambientRhOffsetPct, 1);
    json += ",\"motorModeMaxRpm\":" + String(MetaSense::Settings::motorModeMaxRpm, 0);
    json += ",\"pulsesPerRev\":" + String(MetaSense::Settings::pulsesPerRev, 2);
    json += ",\"pulsesPerRevDrum\":" + String(MetaSense::Settings::pulsesPerRevDrum, 2);
    json += ",\"rpmFilter\":" + String(MetaSense::Settings::filterAlpha, 3);
    json += ",\"mode\":" + String(MetaSense::Settings::inertiaMode ? "\"inertia\"" : "\"brake\"");
    json += ",\"virtGearRatio\":" + String(MetaSense::Settings::virtGearRatio, 3);
    json += ",\"drumMass\":" + String(MetaSense::Settings::drumMassKg, 2);
    json += ",\"drumCm\":" + String(MetaSense::Settings::drumRadiusM * 200.0f, 1);
    json += ",\"drumInertiaType\":\"" + String(MetaSense::Settings::drumWallM > 0.0f ? "hollow" : "solid") + "\"";
    json += ",\"drumWallCm\":" + String(MetaSense::Settings::drumWallM * 100.0f, 1);
    json += ",\"drumInertiaCustom\":" + String(MetaSense::Settings::drumInertiaKgM2, 4);
    json += ",\"brakeToEngineRatio\":" + String(MetaSense::Settings::virtGearRatio, 3);
    json += ",\"drivetrainEff\":" + String(MetaSense::Settings::drivetrainEff, 1);
    json += ",\"rpmSource\":\"leafrpm\"";
    json += ",\"tachoCal\":" + String(MetaSense::Settings::getTachoCal(), 3);
    json += ",\"rpmTarget\":" + String(MetaSense::Settings::getRpmTarget(), 1);
    json += ",\"rpmStart\":" + String(MetaSense::Settings::rpmStart(), 1);
    json += ",\"rpmEnd\":" + String(MetaSense::Settings::rpmEnd(), 1);
    json += ",\"version\":\"3.0\"";
    json += ",\"buildDate\":\"" __DATE__ " " __TIME__ "\"";
    json += "}}";

    return json;
}

void sendProfileSnapshot(AsyncWebSocketClient* client)
{
    const String json = buildProfilePayload(true);

    if (client != nullptr) {
        client->text(json);
    } else {
        MetaSense::WebSocketServer::socket().textAll(json);
    }
}

bool applyProfilePayload(const String& payload)
{
    JsonDocument doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
        return false;
    }

    JsonVariant profile = doc["profile"];
    if (profile.isNull()) {
        profile = doc.as<JsonVariant>();
    }
    if (profile.isNull()) {
        return false;
    }

    if (!profile["calFactor"].isNull()) {
        const float factor = profile["calFactor"].as<float>();
        if (factor > 0.0f) {
            MetaSense::Input::setCalibrationFactor(factor);
            calibrationFactor = factor;
            MetaSense::RunStorage::saveCalibration();
        }
    }

    if (!profile["maxRPM"].isNull()) MetaSense::Settings::maxRPM = profile["maxRPM"].as<float>();
    if (!profile["maxHP"].isNull()) MetaSense::Settings::maxHP = profile["maxHP"].as<float>();
    if (!profile["maxTorque"].isNull()) MetaSense::Settings::maxTorque = profile["maxTorque"].as<float>();
    if (!profile["armCm"].isNull()) MetaSense::Settings::armCm = profile["armCm"].as<float>();
    if (!profile["pulsesPerRev"].isNull()) MetaSense::Settings::pulsesPerRev = profile["pulsesPerRev"].as<float>();
    if (!profile["pulsesPerRevDrum"].isNull()) MetaSense::Settings::pulsesPerRevDrum = profile["pulsesPerRevDrum"].as<float>();
    if (!profile["rpmFilter"].isNull()) MetaSense::Settings::filterAlpha = profile["rpmFilter"].as<float>();
    if (!profile["drivetrainEff"].isNull()) MetaSense::Settings::drivetrainEff = profile["drivetrainEff"].as<float>();
    if (!profile["kpSource"].isNull()) {
        const String kpSource = profile["kpSource"].as<String>();
        MetaSense::Settings::usePot3Kp = (kpSource == "pot3");
    }
    if (!profile["rhOffsetPct"].isNull()) MetaSense::Settings::ambientRhOffsetPct = profile["rhOffsetPct"].as<float>();
    if (!profile["motorModeMaxRpm"].isNull()) MetaSense::Settings::motorModeMaxRpm = profile["motorModeMaxRpm"].as<float>();
    if (!profile["tachoCal"].isNull()) MetaSense::Settings::setTachoCal(profile["tachoCal"].as<float>());
    if (!profile["rpmTarget"].isNull()) MetaSense::Settings::setRpmTarget(profile["rpmTarget"].as<float>());
    if (!profile["rpmStart"].isNull()) MetaSense::Settings::setRpmStart(profile["rpmStart"].as<float>());
    if (!profile["rpmEnd"].isNull()) MetaSense::Settings::setRpmEnd(profile["rpmEnd"].as<float>());

    if (!profile["mode"].isNull()) {
        const String mode = profile["mode"].as<String>();
        MetaSense::Settings::setInertiaMode(mode == "inertia");
    }

    MetaSense::Settings::useCanLeafRpm = true;

    float drumMass = MetaSense::Settings::drumMassKg;
    float drumRadiusM = MetaSense::Settings::drumRadiusM;
    float drumWallM = MetaSense::Settings::drumWallM;
    bool drumParamsChanged = false;

    if (!profile["drumMass"].isNull()) {
        drumMass = profile["drumMass"].as<float>();
        drumParamsChanged = true;
    }
    if (!profile["drumCm"].isNull()) {
        drumRadiusM = (profile["drumCm"].as<float>() / 2.0f) / 100.0f;
        drumParamsChanged = true;
    }
    if (!profile["drumWallCm"].isNull()) {
        drumWallM = profile["drumWallCm"].as<float>() / 100.0f;
        drumParamsChanged = true;
    }
    if (drumParamsChanged) {
        MetaSense::Settings::setDrumParams(drumMass, drumRadiusM, drumWallM);
    }

    float ratio = MetaSense::Settings::virtGearRatio;
    if (!profile["virtGearRatio"].isNull()) {
        ratio = profile["virtGearRatio"].as<float>();
    } else if (!profile["brakeToEngineRatio"].isNull()) {
        ratio = profile["brakeToEngineRatio"].as<float>();
    }
    if (ratio > 0.0f) {
        MetaSense::Settings::virtGearRatio = ratio;
    }

    if (!profile["drumInertiaCustom"].isNull()) {
        const bool customRequested = !profile["drumInertiaType"].isNull() && profile["drumInertiaType"].as<String>() == "custom";
        if (customRequested || profile["drumInertiaType"].isNull()) {
            MetaSense::Settings::setDrumInertiaCustom(profile["drumInertiaCustom"].as<float>());
        }
    }

    MetaSense::Settings::saveToStorage();
    return true;
}

bool saveFactoryProfileToFs()
{
    File file = LittleFS.open(kFactoryProfilePath, "w");
    if (!file) {
        return false;
    }

    const String payload = buildProfilePayload(false);
    const size_t written = file.print(payload);
    file.close();
    return written == payload.length();
}

bool loadFactoryProfileFromFs()
{
    File file = LittleFS.open(kFactoryProfilePath, "r");
    if (!file) {
        return false;
    }

    const String payload = file.readString();
    file.close();
    if (payload.length() == 0) {
        return false;
    }

    return applyProfilePayload(payload);
}

void sendSettingsSnapshot()
{
    calibrationFactor = MetaSense::Input::getCalibrationFactor();

    String json;
    json.reserve(512);

    json = "{\"type\":\"settings\"";
    json += ",\"calFactor\":" + String(calibrationFactor, 6);
    json += ",\"maxRPM\":" + String(MetaSense::Settings::maxRPM, 0);
    json += ",\"maxHP\":" + String(MetaSense::Settings::maxHP, 1);
    json += ",\"maxTorque\":" + String(MetaSense::Settings::maxTorque, 1);
    json += ",\"armCm\":" + String(MetaSense::Settings::armCm, 1);
    json += ",\"kp\":" + String(MetaSense::Settings::kp, 4);
    json += ",\"ki\":" + String(MetaSense::Settings::ki, 4);
    json += ",\"kpSource\":\"" + String(MetaSense::Settings::usePot3Kp ? "pot3" : "firmware") + "\"";
    json += ",\"rhOffsetPct\":" + String(MetaSense::Settings::ambientRhOffsetPct, 1);
    json += ",\"motorModeMaxRpm\":" + String(MetaSense::Settings::motorModeMaxRpm, 0);
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
    json += ",\"rpmSource\":\"leafrpm\"";
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

bool __attribute__((weak)) calibrateWithKnownWeight(float, float&)
{
    return false;
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
        if (!MetaSense::Input::isVcuReady()) {
            MetaSense::WebSocketServer::sendInfo("VCU not ready: run start blocked");
            return;
        }
        MetaSense::DynoStateMachine::startRecording();
        MetaSense::WebSocketServer::sendStatus("Recording started");
    }
    else if (cmdUpper == "STOP") {
        MetaSense::DynoStateMachine::stopRecording();
        MetaSense::WebSocketServer::sendStatus("Recording stopped");
    }
    else if (cmdUpper == "SET_RPM") {
        MetaSense::Settings::setRpmTarget(value);
        MetaSense::Settings::saveToStorage();
        MetaSense::WebSocketServer::sendStatus("RPM target set");
    }
    else if (cmdUpper == "RPM_DIAG") {
        MetaSense::WebSocketServer::sendInfo("RPM diagnostic requested");
    }
    else if (cmdUpper == "SET_TACHO_CAL") {
        float cal = value;
        if (cal == 0.0f && !doc.isNull()) {
            cal = MetaSense::Settings::getTachoCal();
        }
        MetaSense::Settings::setTachoCal(cal);
        MetaSense::Settings::saveToStorage();
        MetaSense::WebSocketServer::sendStatus("Tachogen calibration updated");
    }
    else if (cmdUpper == "SET_TORQUE_FF") {
        MetaSense::DynoStateMachine::setTorqueFeedForward(value);
        MetaSense::WebSocketServer::sendStatus("Torque feed-forward updated");
    }
    else if (cmdUpper == "TARE") {
        MetaSense::Input::tareMainGui();
        calibrationZero = MetaSense::Input::getZeroOffset();
        MetaSense::RunStorage::saveCalibration();
        MetaSense::WebSocketServer::sendStatus("Main tare applied (zero=" + String(calibrationZero, 2) + ")");
    }
    else if (cmdUpper == "CALIBRATE_ZERO") {
        MetaSense::Input::tare();
        calibrationZero = MetaSense::Input::getZeroOffset();
        MetaSense::RunStorage::saveCalibration();
        MetaSense::WebSocketServer::sendStatus("Tare applied (zero=" + String(calibrationZero, 2) + ")");
    }
    else if (isAutoRunCommand) {
        if (!MetaSense::Input::isVcuReady()) {
            MetaSense::WebSocketServer::sendInfo("VCU not ready: auto run blocked");
            return;
        }
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
        MetaSense::Settings::saveToStorage();
        MetaSense::DynoStateMachine::setPanelAuto(true);
        MetaSense::DynoStateMachine::setAutoMode(true);
        MetaSense::DynoStateMachine::startRecording();
        MetaSense::WebSocketServer::sendStatus("Auto run armed");
    }
    else if (cmdUpper == "MANUAL_START") {
        if (!MetaSense::Input::isVcuReady()) {
            MetaSense::WebSocketServer::sendInfo("VCU not ready: manual start blocked");
            return;
        }
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
    else if (cmdUpper.startsWith("CALIBRATE:")) {
        float knownWeightKg = 0.0f;
        if (!parseKnownWeightCommand(cmd, knownWeightKg)) {
            MetaSense::WebSocketServer::sendInfo("Invalid calibration weight");
            return;
        }

        float newFactor = 0.0f;
        if (!MetaSense::Input::calibrateWithKnownWeight(knownWeightKg, newFactor)) {
            MetaSense::WebSocketServer::sendInfo("Calibration failed (tare first, then apply known load)");
            return;
        }

        calibrationFactor = newFactor;
        MetaSense::RunStorage::saveCalibration();
        MetaSense::WebSocketServer::sendStatus("Calibration factor updated (factor=" + String(newFactor, 6) + ")");
    }
    else if (cmdUpper == "GET_CALIBRATION") {
        calibrationZero = MetaSense::Input::getZeroOffset();
        calibrationFactor = MetaSense::Input::getCalibrationFactor();
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
    else if (cmdUpper == "EXPORT_PROFILE") {
        sendProfileSnapshot(client);
        MetaSense::WebSocketServer::sendStatus("Profile exported");
    }
    else if (cmdUpper == "SAVE_FACTORY_PROFILE") {
        if (!saveFactoryProfileToFs()) {
            MetaSense::WebSocketServer::sendInfo("Failed to write factory settings file to FS");
            return;
        }
        MetaSense::WebSocketServer::sendStatus("Factory settings uploaded to FS");
    }
    else if (cmdUpper == "LOAD_FACTORY_PROFILE") {
        MetaSense::WebSocketServer::sendInfo("Factory profile load is disabled; active settings come from NVS.");
    }
    else if (cmdUpper.startsWith("APPLY_PROFILE:")) {
        const String payload = cmd.substring(14);
        if (!applyProfilePayload(payload)) {
            MetaSense::WebSocketServer::sendInfo("Invalid profile payload");
            return;
        }
        sendSettingsSnapshot();
        MetaSense::WebSocketServer::sendStatus("Profile applied");
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
        MetaSense::Settings::resetToDefaults();
        MetaSense::Settings::saveToStorage();
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
            MetaSense::Settings::maxRPM = (fval > 0.0f) ? fval : MetaSense::Settings::maxRPM;
        } else if (key == "maxHP") {
            MetaSense::Settings::maxHP = (fval > 0.0f) ? fval : MetaSense::Settings::maxHP;
        } else if (key == "maxTorque") {
            MetaSense::Settings::maxTorque = (fval > 0.0f) ? fval : MetaSense::Settings::maxTorque;
        } else if (key == "armCm") {
            MetaSense::Settings::armCm = (fval > 0.0f) ? fval : MetaSense::Settings::armCm;
        } else if (key == "kp") {
            MetaSense::Settings::kp = (fval >= 0.0f) ? fval : MetaSense::Settings::kp;
            MetaSense::ControlTask::configurePI(MetaSense::Settings::kp,
                                                MetaSense::Settings::ki,
                                                0.0f,
                                                static_cast<float>(MetaSense::Settings::maxTorque));
        } else if (key == "ki") {
            MetaSense::Settings::ki = (fval >= 0.0f) ? fval : MetaSense::Settings::ki;
            MetaSense::ControlTask::configurePI(MetaSense::Settings::kp,
                                                MetaSense::Settings::ki,
                                                0.0f,
                                                static_cast<float>(MetaSense::Settings::maxTorque));
        } else if (key == "kpSource") {
            MetaSense::Settings::usePot3Kp = (val == "pot3");
            if (!MetaSense::Settings::usePot3Kp) {
                MetaSense::ControlTask::configurePI(MetaSense::Settings::kp,
                                                    MetaSense::Settings::ki,
                                                    0.0f,
                                                    static_cast<float>(MetaSense::Settings::maxTorque));
            }
        } else if (key == "rhOffsetPct") {
            if (fval < -50.0f) {
                MetaSense::Settings::ambientRhOffsetPct = -50.0f;
            } else if (fval > 50.0f) {
                MetaSense::Settings::ambientRhOffsetPct = 50.0f;
            } else {
                MetaSense::Settings::ambientRhOffsetPct = fval;
            }
        } else if (key == "motorModeMaxRpm") {
            MetaSense::Settings::motorModeMaxRpm = (fval > 0.0f) ? fval : MetaSense::Settings::motorModeMaxRpm;
        } else if (key == "pulsesPerRev") {
            MetaSense::Settings::pulsesPerRev = (fval > 0.0f) ? fval : MetaSense::Settings::pulsesPerRev;
        } else if (key == "pulsesPerRevDrum") {
            MetaSense::Settings::pulsesPerRevDrum = (fval > 0.0f) ? fval : MetaSense::Settings::pulsesPerRevDrum;
        } else if (key == "drivetrainEff") {
            MetaSense::Settings::drivetrainEff = (fval > 0.0f && fval <= 100.0f) ? fval : MetaSense::Settings::drivetrainEff;
        } else if (key == "rpmSource") {
            MetaSense::Settings::useCanLeafRpm = true;
        } else if (key == "brakeToEngineRatio") {
            MetaSense::Settings::virtGearRatio = (fval > 0.0f) ? fval : MetaSense::Settings::virtGearRatio;
        } else if (key == "virtGearRatio") {
            MetaSense::Settings::virtGearRatio = (fval > 0.0f) ? fval : MetaSense::Settings::virtGearRatio;
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
        MetaSense::Settings::saveToStorage();
        sendSettingsSnapshot();
        MetaSense::WebSocketServer::sendStatus(key + " saved");
    }
    else {
        MetaSense::WebSocketServer::sendInfo("Unknown command: " + cmd);
    }
}

bool loadFactoryProfileOnBoot()
{
    // Settings now persist directly in NVS; skip boot-time factory profile import.
    return false;
}

} // namespace MetaSense::CommandRouter
