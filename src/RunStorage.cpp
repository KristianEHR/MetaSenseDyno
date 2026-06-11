#include "RunStorage.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <time.h>
#include <vector>
#include <algorithm>

namespace {

// Returns a valid Unix timestamp if NTP has synced, otherwise uptime seconds.
uint32_t getEpochTime()
{
    time_t now = time(nullptr);
    return (now > 1000000000L) ? (uint32_t)now : (uint32_t)(millis() / 1000);
}

MetaSense::Telemetry lastRun;
const char* RUNS_DIR = "/runs";

// Returns sorted list of run filenames (base names only, e.g. "run_000.json")
std::vector<String> sortedRunFiles()
{
    std::vector<String> names;
    if (!LittleFS.exists(RUNS_DIR)) return names;

    File dir = LittleFS.open(RUNS_DIR);
    if (!dir || !dir.isDirectory()) return names;

    File entry;
    while ((entry = dir.openNextFile())) {
        if (!entry.isDirectory()) {
            String name = String(entry.name());
            // entry.name() may return full path; keep only the base name
            int slash = name.lastIndexOf('/');
            if (slash >= 0) name = name.substring(slash + 1);
            if (name.endsWith(".json")) names.push_back(name);
        }
        entry.close();
    }
    dir.close();

    std::sort(names.begin(), names.end());
    return names;
}

} // anonymous namespace


namespace MetaSense::RunStorage {

void save(const MetaSense::Telemetry& telemetry)
{
    lastRun = telemetry;
}

const MetaSense::Telemetry& latest()
{
    return lastRun;
}

void flush() {}

void saveCalibration() {}

// ─── Persistent run history ─────────────────────────────────────────────────

bool saveRun(const String& payload)
{
    if (!LittleFS.exists(RUNS_DIR)) LittleFS.mkdir(RUNS_DIR);

    // Find the next available run_NNN.json filename
    char path[40];
    int idx = 0;
    for (; idx <= 999; idx++) {
        snprintf(path, sizeof(path), "%s/run_%03d.json", RUNS_DIR, idx);
        if (!LittleFS.exists(path)) break;
        if (idx == 999) return false;
    }

    JsonDocument doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) return false;

    JsonDocument stored;
    stored["ts"] = getEpochTime();
    stored["customer"] = doc["customer"] | "";
    stored["vehicle"]  = doc["unit"]     | "";
    stored["comments"] = doc["comments"] | "";

    JsonObject peaks   = doc["peaks"].as<JsonObject>();
    stored["hp"]       = peaks["hp"]         | 0.0f;
    stored["hp_rpm"]   = peaks["hp_rpm"]      | 0.0f;
    stored["torque"]   = peaks["torque"]      | 0.0f;
    stored["t_rpm"]    = peaks["torque_rpm"]  | 0.0f;
    stored["egt"]      = peaks["egt"]         | 0.0f;

    JsonArray dstPts = stored["points"].to<JsonArray>();
    for (JsonObject p : doc["points"].as<JsonArray>()) {
        JsonObject np = dstPts.add<JsonObject>();
        np["r"] = p["r"]; np["h"] = p["h"]; np["t"] = p["t"];
    }

    File f = LittleFS.open(path, "w");
    if (!f) return false;
    serializeJson(stored, f);
    f.close();
    return true;
}

String listRuns()
{
    auto names = sortedRunFiles();

    String out = "[";
    bool first = true;

    for (const String& fname : names) {
        String fpath = String(RUNS_DIR) + "/" + fname;
        File f = LittleFS.open(fpath, "r");
        if (!f) continue;

        // Use a filter to skip the (large) points array — only read metadata
        JsonDocument filter;
        filter["ts"]       = true;
        filter["customer"] = true;
        filter["vehicle"]  = true;
        filter["hp"]       = true;
        filter["torque"]   = true;
        filter["egt"]      = true;

        JsonDocument meta;
        deserializeJson(meta, f, DeserializationOption::Filter(filter));
        f.close();

        if (!first) out += ",";
        first = false;

        // Escape customer/vehicle strings to avoid breaking JSON
        String customer = String((const char*)(meta["customer"] | ""));
        String vehicle  = String((const char*)(meta["vehicle"]  | ""));
        customer.replace("\"", "\\\"");
        vehicle.replace("\"", "\\\"");

        out += "{\"filename\":\"" + fname + "\"";
        out += ",\"ts\":"         + String((long)(meta["ts"] | 0));
        out += ",\"customer\":\"" + customer + "\"";
        out += ",\"vehicle\":\""  + vehicle  + "\"";
        out += ",\"hp\":"         + String((float)(meta["hp"]     | 0.0f), 2);
        out += ",\"torque\":"     + String((float)(meta["torque"] | 0.0f), 2);
        out += ",\"egt\":"        + String((float)(meta["egt"]    | 0.0f), 1);
        out += "}";
    }

    out += "]";
    return out;
}

String loadRunPoints(const String& filename)
{
    // Guard against path traversal
    if (filename.indexOf('/') >= 0 || filename.indexOf("..") >= 0) return "[]";

    String fpath = String(RUNS_DIR) + "/" + filename;
    File f = LittleFS.open(fpath, "r");
    if (!f) return "[]";

    JsonDocument filter;
    filter["points"] = true;

    JsonDocument doc;
    deserializeJson(doc, f, DeserializationOption::Filter(filter));
    f.close();

    String out;
    serializeJson(doc["points"], out);
    return out;
}

bool deleteRunByIndex(int index)
{
    auto names = sortedRunFiles();
    if (index < 0 || index >= (int)names.size()) return false;
    String fpath = String(RUNS_DIR) + "/" + names[index];
    return LittleFS.remove(fpath);
}

} // namespace MetaSense::RunStorage

