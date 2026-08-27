#include <Arduino.h>
#include <ArduinoJson.h>
#include <sys/time.h>
#include <Preferences.h>
#include <LittleFS.h>

#include <ESPAsyncWebServer.h>
#include "DynoStateMachine.h"
#include "HardwareOutputStateMachine.h"
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
bool requestCalibrationWithKnownWeight(float knownWeightKg);
float getCalibrationFactor();
float getZeroOffset();
float getZeroOffsetRaw();
bool isVcuReady();
void setUiModeHintTrend(bool trendMode);
bool isUiModeHintTrend();
bool applyLoadCellSettingsProfile();
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
constexpr float kDefaultCalibrationKnownWeightKg = 3.404f;

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
    calibrationZero = MetaSense::Input::getZeroOffsetRaw();

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
    json += ",\"idleTorqueNm\":" + String(MetaSense::Settings::idleTorqueNm, 2);
    json += ",\"brakeMaxTorqueNm\":" + String(MetaSense::Settings::brakeMaxTorqueNm, 1);
    json += ",\"leafSimFeedback\":" + String(MetaSense::Settings::leafSimFeedbackEnabled ? 1 : 0);
    json += ",\"pulsesPerRev\":" + String(MetaSense::Settings::pulsesPerRev, 2);
    json += ",\"pulsesPerRevDrum\":" + String(MetaSense::Settings::pulsesPerRevDrum, 2);
    json += ",\"rpmFilter\":" + String(MetaSense::Settings::filterAlpha, 3);
    json += ",\"loadAvgN\":" + String(MetaSense::Settings::loadAvgN, 0);
    json += ",\"loadAvgN2\":" + String(MetaSense::Settings::loadAvgN2, 0);
    json += ",\"loadFilterMode\":\"" + String(MetaSense::Settings::loadFilterMode == 1 ? "two_stage_ma" : "moving_avg") + "\"";
    json += ",\"loadCellGain\":" + String(MetaSense::Settings::loadCellGain);
    json += ",\"loadCellRateSps\":" + String(MetaSense::Settings::loadCellRateSps);
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
    if (!profile["loadAvgN"].isNull()) {
        const float n = profile["loadAvgN"].as<float>();
        if (n < 1.0f) {
            MetaSense::Settings::loadAvgN = 1.0f;
        } else if (n > 255.0f) {
            MetaSense::Settings::loadAvgN = 255.0f;
        } else {
            MetaSense::Settings::loadAvgN = n;
        }
    }
    if (!profile["loadAvgN2"].isNull()) {
        const float n = profile["loadAvgN2"].as<float>();
        if (n < 1.0f) {
            MetaSense::Settings::loadAvgN2 = 1.0f;
        } else if (n > 255.0f) {
            MetaSense::Settings::loadAvgN2 = 255.0f;
        } else {
            MetaSense::Settings::loadAvgN2 = n;
        }
    }
    if (!profile["loadFilterMode"].isNull()) {
        const String mode = profile["loadFilterMode"].as<String>();
        if (mode.length() > 0) {
            MetaSense::Settings::loadFilterMode =
                (mode == "two_stage_ma" || mode == "b") ? 1 : 0;
        } else {
            const int modeInt = profile["loadFilterMode"].as<int>();
            MetaSense::Settings::loadFilterMode = (modeInt == 1) ? 1 : 0;
        }
    }
    if (!profile["loadCellGain"].isNull()) {
        const uint16_t gain = static_cast<uint16_t>(profile["loadCellGain"].as<int>());
        if (gain == 1 || gain == 2 || gain == 4 || gain == 8 || gain == 16 || gain == 32 || gain == 64 || gain == 128) {
            MetaSense::Settings::loadCellGain = gain;
        }
    }
    if (!profile["loadCellRateSps"].isNull()) {
        const uint16_t rate = static_cast<uint16_t>(profile["loadCellRateSps"].as<int>());
        if (rate == 10 || rate == 20 || rate == 40 || rate == 80 || rate == 320) {
            MetaSense::Settings::loadCellRateSps = rate;
        }
    }
    if (!profile["drivetrainEff"].isNull()) MetaSense::Settings::drivetrainEff = profile["drivetrainEff"].as<float>();
    if (!profile["kpSource"].isNull()) {
        const String kpSource = profile["kpSource"].as<String>();
        MetaSense::Settings::usePot3Kp = (kpSource == "pot3");
    }
    if (!profile["rhOffsetPct"].isNull()) MetaSense::Settings::ambientRhOffsetPct = profile["rhOffsetPct"].as<float>();
    if (!profile["motorModeMaxRpm"].isNull()) MetaSense::Settings::motorModeMaxRpm = profile["motorModeMaxRpm"].as<float>();
    if (!profile["idleTorqueNm"].isNull()) {
        const float idleTorque = profile["idleTorqueNm"].as<float>();
        if (idleTorque < 0.0f) {
            MetaSense::Settings::idleTorqueNm = 0.0f;
        } else if (idleTorque > 10.0f) {
            MetaSense::Settings::idleTorqueNm = 10.0f;
        } else {
            MetaSense::Settings::idleTorqueNm = idleTorque;
        }
    }
    if (!profile["brakeMaxTorqueNm"].isNull()) {
        const float brakeMaxTorque = profile["brakeMaxTorqueNm"].as<float>();
        if (brakeMaxTorque < 0.0f) {
            MetaSense::Settings::brakeMaxTorqueNm = 0.0f;
        } else if (brakeMaxTorque > 200.0f) {
            MetaSense::Settings::brakeMaxTorqueNm = 200.0f;
        } else {
            MetaSense::Settings::brakeMaxTorqueNm = brakeMaxTorque;
        }
    }
    if (!profile["leafSimFeedback"].isNull()) {
        if (profile["leafSimFeedback"].is<bool>()) {
            MetaSense::Settings::leafSimFeedbackEnabled = profile["leafSimFeedback"].as<bool>();
        } else {
            String flag = profile["leafSimFeedback"].as<String>();
            flag.trim();
            flag.toLowerCase();
            MetaSense::Settings::leafSimFeedbackEnabled =
                (flag == "1" || flag == "true" || flag == "on" || flag == "yes");
        }
    }
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
    json += ",\"idleTorqueNm\":" + String(MetaSense::Settings::idleTorqueNm, 2);
    json += ",\"brakeMaxTorqueNm\":" + String(MetaSense::Settings::brakeMaxTorqueNm, 1);
    json += ",\"forceVcuReadyForUiTest\":" + String(MetaSense::Settings::forceVcuReadyForUiTest ? 1 : 0);
    json += ",\"leafSimFeedback\":" + String(MetaSense::Settings::leafSimFeedbackEnabled ? 1 : 0);
    json += ",\"pulsesPerRev\":" + String(MetaSense::Settings::pulsesPerRev, 2);
    json += ",\"rpmFilter\":" + String(MetaSense::Settings::filterAlpha, 3);
    json += ",\"loadAvgN\":" + String(MetaSense::Settings::loadAvgN, 0);
    json += ",\"loadAvgN2\":" + String(MetaSense::Settings::loadAvgN2, 0);
    json += ",\"loadFilterMode\":\"" + String(MetaSense::Settings::loadFilterMode == 1 ? "two_stage_ma" : "moving_avg") + "\"";
    json += ",\"loadCellGain\":" + String(MetaSense::Settings::loadCellGain);
    json += ",\"loadCellRateSps\":" + String(MetaSense::Settings::loadCellRateSps);
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

    // Fast-path large run payloads to avoid unnecessary uppercase copies
    // and JSON probe parsing on SAVE_RUN_DATA messages.
    String rawMsg = msg;
    rawMsg.trim();
    if (rawMsg.startsWith("SAVE_RUN_DATA:")) {
        const String payload = rawMsg.substring(14);
        if (MetaSense::RunStorage::saveRun(payload)) {
            MetaSense::WebSocketServer::sendStatus("Run saved");
        } else {
            MetaSense::WebSocketServer::sendInfo("Run save failed");
        }
        return;
    }
    #pragma GCC diagnostic push
#   pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    StaticJsonDocument<256> doc;
    #pragma GCC diagnostic pop

    if (!deserializeJson(doc, rawMsg)) {
        cmd = String((const char*)(doc["cmd"] | ""));
        value = doc["value"] | 0.0f;
    } else {
        cmd = rawMsg;
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
        if (MetaSense::Input::requestTare()) {
            MetaSense::WebSocketServer::sendStatus("Tare requested");
        } else {
            MetaSense::WebSocketServer::sendInfo("Tare/calibration already in progress");
        }
    }
    else if (cmdUpper == "CALIBRATE_ZERO") {
        if (MetaSense::Input::requestTare()) {
            MetaSense::WebSocketServer::sendStatus("Tare requested");
        } else {
            MetaSense::WebSocketServer::sendInfo("Tare/calibration already in progress");
        }
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

        const int thirdColon = cmd.indexOf(':', secondColon + 1);

        const float startRpm = cmd.substring(firstColon + 1, secondColon).toFloat();
        const float endRpm = (thirdColon > secondColon)
            ? cmd.substring(secondColon + 1, thirdColon).toFloat()
            : cmd.substring(secondColon + 1).toFloat();
        if (startRpm <= 0.0f || endRpm <= startRpm) {
            MetaSense::WebSocketServer::sendInfo("Invalid AUTO_RUN range");
            return;
        }

        MetaSense::DynoStateMachine::AutoRampProfile rampProfile =
            MetaSense::DynoStateMachine::AutoRampProfile::Hybrid;
        if (thirdColon > secondColon) {
            String profile = cmd.substring(thirdColon + 1);
            profile.trim();
            profile.toLowerCase();
            if (profile == "linear") {
                rampProfile = MetaSense::DynoStateMachine::AutoRampProfile::Linear;
            } else if (profile == "exponential" || profile == "exp") {
                rampProfile = MetaSense::DynoStateMachine::AutoRampProfile::Exponential;
            } else if (profile == "hybrid") {
                rampProfile = MetaSense::DynoStateMachine::AutoRampProfile::Hybrid;
            }
        }

        MetaSense::Settings::setRpmStart(startRpm);
        MetaSense::Settings::setRpmEnd(endRpm);
        MetaSense::Settings::saveToStorage();
        MetaSense::DynoStateMachine::setAutoRampProfile(rampProfile);
        MetaSense::DynoStateMachine::setPanelAuto(true);
        MetaSense::DynoStateMachine::setAutoMode(true);
        MetaSense::DynoStateMachine::startRecording();
        MetaSense::WebSocketServer::sendStatus("Auto run armed (" + String(MetaSense::DynoStateMachine::autoRampProfileName()) + ")");
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
    }
    else if (cmdUpper == "CANCEL_AUTO_RUN") {
        MetaSense::DynoStateMachine::setAutoMode(false);
        MetaSense::DynoStateMachine::setPanelAuto(false);
        MetaSense::DynoStateMachine::abortAutoRun();
        MetaSense::WebSocketServer::sendStatus("Auto run cancelled");
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
        float knownWeightKg = kDefaultCalibrationKnownWeightKg;
        if (!parseKnownWeightCommand(cmd, knownWeightKg)) {
            knownWeightKg = kDefaultCalibrationKnownWeightKg;
            MetaSense::WebSocketServer::sendInfo("Calibration weight defaulted to 3.404 kg");
        }

        if (!MetaSense::Input::requestCalibrationWithKnownWeight(knownWeightKg)) {
            MetaSense::WebSocketServer::sendInfo("Calibration busy or load cell unavailable");
            return;
        }

        MetaSense::WebSocketServer::sendStatus("Calibration queued");
    }
    else if (cmdUpper == "GET_CALIBRATION") {
        calibrationZero = MetaSense::Input::getZeroOffsetRaw();
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
        MetaSense::Input::setUiModeHintTrend(false);
    }
    else if (cmdUpper.startsWith("SET_UI_MODE:")) {
        String mode = cmd.substring(12);
        mode.trim();
        mode.toUpperCase();
        const bool trendMode = (mode == "TREND");
        MetaSense::Input::setUiModeHintTrend(trendMode);
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
        bool persistSettings = true;
        bool applyLoadCellProfile = false;

        // Safety check: prevent manual torque commands during INIT state
        const char* hwState = MetaSense::HardwareOutputStateMachine::stateName();
        const bool isInitState = (hwState != nullptr) && (strcmp(hwState, "INIT") == 0);
        
        if (key == "leaf1d4TorqueNm") {
            if (!isInitState) {
                MetaSense::Input::setLeafUiTorqueDemandNm(fval);
            } else {
                Serial.printf("[SAFETY] Manual torque command rejected during INIT state\n");
            }
            persistSettings = false;
        } else if (key == "leaf1d4TorqueInc") {
            // Increment torque by +0.25 Nm (only in IDLE/MOTOR states)
            if (!isInitState) {
                float current = MetaSense::Input::getLeafUiTorqueDemandNm();
                MetaSense::Input::setLeafUiTorqueDemandNm(current + 0.25f);
            } else {
                Serial.printf("[SAFETY] Manual torque inc rejected during INIT state\n");
            }
            persistSettings = false;
        } else if (key == "leaf1d4TorqueDec") {
            // Decrement torque by -0.25 Nm (only in IDLE/MOTOR states)
            if (!isInitState) {
                float current = MetaSense::Input::getLeafUiTorqueDemandNm();
                MetaSense::Input::setLeafUiTorqueDemandNm(current - 0.25f);
            } else {
                Serial.printf("[SAFETY] Manual torque dec rejected during INIT state\n");
            }
            persistSettings = false;
        } else if (key == "leaf1d4TorqueMode") {
            // Set manual/auto mode: "manual" or "auto" (only in IDLE/MOTOR states)
            if (!isInitState) {
                MetaSense::Input::setLeafManualTorqueMode(val == "manual");
            } else {
                Serial.printf("[SAFETY] Manual torque mode change rejected during INIT state\n");
            }
            persistSettings = false;
        } else if (key == "leaf11aGear") {
            // Set 0x11A gear position (0-15)
            const uint8_t gearVal = static_cast<uint8_t>(val.toInt());
            MetaSense::Input::setLeaf11aUiGear(gearVal);
            persistSettings = false;
        } else if (key == "leaf11aCarOnOff") {
            // Set 0x11A car on/off status (0-7)
            const uint8_t carOnOffVal = static_cast<uint8_t>(val.toInt());
            MetaSense::Input::setLeaf11aUiCarOnOff(carOnOffVal);
            persistSettings = false;
        } else if (key == "rpmFilter") {
            MetaSense::Settings::filterAlpha = fval;
        } else if (key == "loadAvgN") {
            if (fval < 1.0f) {
                MetaSense::Settings::loadAvgN = 1.0f;
            } else if (fval > 255.0f) {
                MetaSense::Settings::loadAvgN = 255.0f;
            } else {
                MetaSense::Settings::loadAvgN = fval;
            }
        } else if (key == "loadAvgN2") {
            if (fval < 1.0f) {
                MetaSense::Settings::loadAvgN2 = 1.0f;
            } else if (fval > 255.0f) {
                MetaSense::Settings::loadAvgN2 = 255.0f;
            } else {
                MetaSense::Settings::loadAvgN2 = fval;
            }
        } else if (key == "loadFilterMode") {
            MetaSense::Settings::loadFilterMode =
                (val == "two_stage_ma" || val == "b" || val == "1") ? 1 : 0;
        } else if (key == "loadCellGain") {
            const uint16_t gain = static_cast<uint16_t>(val.toInt());
            if (gain == 1 || gain == 2 || gain == 4 || gain == 8 || gain == 16 || gain == 32 || gain == 64 || gain == 128) {
                MetaSense::Settings::loadCellGain = gain;
                applyLoadCellProfile = true;
            }
        } else if (key == "loadCellRateSps") {
            const uint16_t rate = static_cast<uint16_t>(val.toInt());
            if (rate == 10 || rate == 20 || rate == 40 || rate == 80 || rate == 320) {
                MetaSense::Settings::loadCellRateSps = rate;
                applyLoadCellProfile = true;
            }
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
        } else if (key == "idleTorqueNm") {
            if (fval < 0.0f) {
                MetaSense::Settings::idleTorqueNm = 0.0f;
            } else if (fval > 10.0f) {
                MetaSense::Settings::idleTorqueNm = 10.0f;
            } else {
                MetaSense::Settings::idleTorqueNm = fval;
            }
        } else if (key == "brakeMaxTorqueNm") {
            if (fval < 0.0f) {
                MetaSense::Settings::brakeMaxTorqueNm = 0.0f;
            } else if (fval > 200.0f) {
                MetaSense::Settings::brakeMaxTorqueNm = 200.0f;
            } else {
                MetaSense::Settings::brakeMaxTorqueNm = fval;
            }
        } else if (key == "forceVcuReadyForUiTest") {
            String flag = val;
            flag.trim();
            flag.toLowerCase();
            MetaSense::Settings::forceVcuReadyForUiTest =
                (flag == "1" || flag == "true" || flag == "on" || flag == "yes");
        } else if (key == "leafSimFeedback") {
            String flag = val;
            flag.trim();
            flag.toLowerCase();
            MetaSense::Settings::leafSimFeedbackEnabled =
                (flag == "1" || flag == "true" || flag == "on" || flag == "yes");
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
        if (persistSettings) {
            MetaSense::Settings::saveToStorage();
            if (applyLoadCellProfile) {
                const bool applied = MetaSense::Input::applyLoadCellSettingsProfile();
                if (!applied) {
                    MetaSense::WebSocketServer::sendInfo("Load-cell profile saved but NAU apply failed (will apply on next init)");
                }
            }
            sendSettingsSnapshot();
        }
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
