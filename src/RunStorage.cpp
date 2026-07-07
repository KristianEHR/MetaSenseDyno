#include "RunStorage.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <esp_timer.h>

#include "Input.h"

namespace {

// Returns a valid Unix timestamp if NTP has synced, otherwise uptime seconds.
uint32_t getEpochTime()
{
    time_t now = time(nullptr);
    return (now > 1000000000L) ? (uint32_t)now : (uint32_t)(millis() / 1000);
}

portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
MetaSense::Telemetry lastRun;
uint32_t telemetryVersion = 0;
TaskHandle_t publishTaskHandle = nullptr;
const char* RUNS_DIR = "/runs";
const char* CAPTURES_DIR = "/captures";

struct RawCaptureState {
    bool active = false;
    File file;
    String filePath;
    String reportPath;
    String buffer;
    uint64_t startUs = 0;
    uint64_t endUs = 0;
    uint32_t startMs = 0;
    uint32_t endMs = 0;
    float baselineKg = 0.0f;
    float testLoadKg = 0.0f;
    uint32_t sampleCount = 0;
    uint64_t lastSampleUs = 0;
    double sum = 0.0;
    double sumSquares = 0.0;
    float minRaw = 0.0f;
    float maxRaw = 0.0f;
    float prevRaw = 0.0f;
    bool havePrev = false;
    float maxAbsDelta = 0.0f;
    double deltaSum = 0.0;
    double deltaSumSquares = 0.0;
    uint32_t deltaCount = 0;
    uint16_t expectedSps = 0;
    uint64_t expectedPeriodUs = 0;
    uint32_t gapCount = 0;
    uint64_t maxGapUs = 0;
    uint32_t zeroDeltaCount = 0;
    double elapsedSum = 0.0;
    double elapsedSumSquares = 0.0;
    double elapsedRawSum = 0.0;
    double elapsedRawProductSum = 0.0;
    String lastStopReason = "idle";
    uint64_t lastFinalizeUs = 0;
    uint32_t lastDurationMs = 0;
    String lastFilePath;
};

struct FsLiveProbeState {
    bool active = false;
    File file;
    String filePath;
    String buffer;
    uint64_t startUs = 0;
    uint64_t endUs = 0;
    uint32_t sampleCount = 0;
    uint64_t lastSampleUs = 0;
    String lastStopReason = "idle";
    uint64_t lastFinalizeUs = 0;
    uint32_t lastDurationMs = 0;
    String lastFilePath;
};

portMUX_TYPE rawCaptureMux = portMUX_INITIALIZER_UNLOCKED;
RawCaptureState rawCapture;
bool rawCaptureStartPending = false;
portMUX_TYPE fsLiveProbeMux = portMUX_INITIALIZER_UNLOCKED;
FsLiveProbeState fsLiveProbe;
uint32_t lastRunPayloadHash = 0;
uint32_t lastRunPayloadMs = 0;
constexpr uint32_t kRunSaveDedupWindowMs = 5000U;

uint32_t fnv1a32(const String& text)
{
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < text.length(); ++i) {
        hash ^= static_cast<uint8_t>(text[i]);
        hash *= 16777619UL;
    }
    return hash;
}

void logRawCaptureEvent(const char* tag,
                        const String& filePath,
                        uint64_t startUs,
                        uint64_t endUs,
                        uint64_t nowUs,
                        uint32_t samples,
                        const String& reason)
{
    const unsigned long long startMs = static_cast<unsigned long long>(startUs / 1000ULL);
    const unsigned long long endMs = static_cast<unsigned long long>(endUs / 1000ULL);
    const unsigned long long nowMs = static_cast<unsigned long long>(nowUs / 1000ULL);
    const unsigned long long durMs = (endUs > startUs) ? static_cast<unsigned long long>((endUs - startUs) / 1000ULL) : 0ULL;
    Serial.printf("[RAW] %s file=%s startMs=%llu endMs=%llu nowMs=%llu durMs=%llu samples=%lu reason=%s\n",
                  tag,
                  filePath.c_str(),
                  startMs,
                  endMs,
                  nowMs,
                  durMs,
                  static_cast<unsigned long>(samples),
                  reason.c_str());
    Serial0.printf("[RAW] %s file=%s startMs=%llu endMs=%llu nowMs=%llu durMs=%llu samples=%lu reason=%s\n",
                   tag,
                   filePath.c_str(),
                   startMs,
                   endMs,
                   nowMs,
                   durMs,
                   static_cast<unsigned long>(samples),
                   reason.c_str());
}

String makeCaptureBaseName(uint32_t epoch)
{
    return String("raw_") + String(epoch);
}

String makeProbeBaseName(uint32_t epoch)
{
    return String("fs_probe_") + String(epoch);
}

String captureCsvPathForName(const String& name)
{
    String path = String(CAPTURES_DIR) + "/" + name;
    if (LittleFS.exists(path)) {
        return path;
    }

    path = String("/") + name;
    if (LittleFS.exists(path)) {
        return path;
    }

    return "";
}

String captureReportPathForBase(const String& base)
{
    String path = String(CAPTURES_DIR) + "/" + base + ".json";
    if (LittleFS.exists(path)) {
        return path;
    }

    const String rootPath = String("/") + base + ".json";
    if (LittleFS.exists(rootPath)) {
        return rootPath;
    }

    const String captureCsvPath = String(CAPTURES_DIR) + "/" + base + ".csv";
    if (LittleFS.exists(captureCsvPath)) {
        return path;
    }

    return rootPath;
}

bool trySynthesizeCaptureReportFromCsv(const String& filename)
{
    if (filename.indexOf('/') >= 0 || filename.indexOf("..") >= 0 || !filename.endsWith(".csv")) {
        return false;
    }

    String base = filename;
    base.remove(base.length() - 4);

    const String csvPath = captureCsvPathForName(filename);
    if (csvPath.isEmpty()) {
        return false;
    }

    bool activeMatches = false;
    portENTER_CRITICAL(&rawCaptureMux);
    activeMatches = rawCapture.active && rawCapture.filePath.endsWith(filename);
    portEXIT_CRITICAL(&rawCaptureMux);
    if (activeMatches) {
        return false;
    }

    const String reportPath = captureReportPathForBase(base);
    if (LittleFS.exists(reportPath)) {
        return true;
    }

    File csv = LittleFS.open(csvPath, "r");
    if (!csv) {
        return false;
    }

    float baselineKg = 0.0f;
    float testLoadKg = 0.0f;
    uint32_t durationHintMs = 0;

    uint32_t sampleCount = 0;
    double sum = 0.0;
    double sumSquares = 0.0;
    float minRaw = 0.0f;
    float maxRaw = 0.0f;
    float prevRaw = 0.0f;
    bool havePrev = false;
    float maxAbsDelta = 0.0f;
    double deltaSum = 0.0;
    double deltaSumSquares = 0.0;
    uint32_t deltaCount = 0;
    uint32_t zeroDeltaCount = 0;
    uint64_t firstTsUs = 0;
    uint64_t lastTsUs = 0;

    while (csv.available()) {
        String line = csv.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) {
            continue;
        }

        if (line.startsWith("# baselineKg=")) {
            baselineKg = line.substring(13).toFloat();
            continue;
        }
        if (line.startsWith("# testLoadKg=")) {
            testLoadKg = line.substring(13).toFloat();
            continue;
        }
        if (line.startsWith("# durationMs=")) {
            durationHintMs = static_cast<uint32_t>(line.substring(13).toInt());
            continue;
        }
        if (line.startsWith("#") || line.startsWith("timestamp_us")) {
            continue;
        }

        const int comma1 = line.indexOf(',');
        if (comma1 <= 0) {
            continue;
        }
        const int comma2 = line.indexOf(',', comma1 + 1);
        if (comma2 <= comma1) {
            continue;
        }

        const String tsStr = line.substring(0, comma1);
        const int comma3 = line.indexOf(',', comma2 + 1);
        const String rawStr = (comma3 > comma2)
            ? line.substring(comma2 + 1, comma3)
            : line.substring(comma2 + 1);
        const uint64_t tsUs = static_cast<uint64_t>(strtoull(tsStr.c_str(), nullptr, 10));
        const float rawValue = rawStr.toFloat();

        if (sampleCount == 0) {
            firstTsUs = tsUs;
            minRaw = rawValue;
            maxRaw = rawValue;
        } else {
            if (rawValue < minRaw) minRaw = rawValue;
            if (rawValue > maxRaw) maxRaw = rawValue;

            const float delta = fabsf(rawValue - prevRaw);
            if (delta == 0.0f) {
                zeroDeltaCount++;
            }
            deltaSum += delta;
            deltaSumSquares += static_cast<double>(delta) * static_cast<double>(delta);
            deltaCount++;
            if (delta > maxAbsDelta) maxAbsDelta = delta;
        }

        lastTsUs = tsUs;
        prevRaw = rawValue;
        havePrev = true;
        sampleCount++;
        sum += static_cast<double>(rawValue);
        sumSquares += static_cast<double>(rawValue) * static_cast<double>(rawValue);
    }

    csv.close();

    const uint32_t durationMs = (sampleCount > 1 && lastTsUs >= firstTsUs)
        ? static_cast<uint32_t>((lastTsUs - firstTsUs) / 1000ULL)
        : durationHintMs;
    const double durationSec = static_cast<double>(durationMs) / 1000.0;
    const double sampleRate = (durationSec > 0.0)
        ? static_cast<double>(sampleCount) / durationSec
        : 0.0;
    const double mean = (sampleCount > 0)
        ? (sum / static_cast<double>(sampleCount))
        : 0.0;

    double variance = 0.0;
    if (sampleCount > 0) {
        variance = (sumSquares / static_cast<double>(sampleCount)) - (mean * mean);
        if (variance < 0.0) variance = 0.0;
    }
    const double stddev = sqrt(variance);
    const double approxSnrDb = (stddev > 0.0 && fabs(mean) > 0.0)
        ? 20.0 * log10(fabs(mean) / stddev)
        : 0.0;
    const double meanDelta = (deltaCount > 0)
        ? (deltaSum / static_cast<double>(deltaCount))
        : 0.0;
    const double deltaRms = (deltaCount > 0)
        ? sqrt(deltaSumSquares / static_cast<double>(deltaCount))
        : 0.0;

    double qualityScore = (sampleCount > 0) ? 75.0 : 0.0;
    if (sampleCount > 0 && stddev > 0.0 && fabs(mean) > 1e-9) {
        const double relativeNoise = stddev / fabs(mean);
        qualityScore -= min(30.0, relativeNoise * 80.0);
    }
    if (qualityScore < 0.0) qualityScore = 0.0;

    String qualityLabel = "recovered";
    if (sampleCount == 0) {
        qualityLabel = "empty";
    } else if (qualityScore >= 85.0) {
        qualityLabel = "good";
    } else if (qualityScore >= 60.0) {
        qualityLabel = "fair";
    } else {
        qualityLabel = "poor";
    }

    File report = LittleFS.open(reportPath, "w");
    if (!report) {
        return false;
    }

    JsonDocument doc;
    doc["file"] = csvPath;
    doc["report"] = reportPath;
    doc["baselineKg"] = baselineKg;
    doc["testLoadKg"] = testLoadKg;
    doc["expectedSps"] = 0;
    doc["expectedPeriodUs"] = 0;
    doc["sampleCount"] = sampleCount;
    doc["durationMs"] = durationMs;
    doc["sampleRate"] = sampleRate;
    doc["meanRaw"] = mean;
    doc["stddevRaw"] = stddev;
    doc["minRaw"] = sampleCount > 0 ? minRaw : 0.0f;
    doc["maxRaw"] = sampleCount > 0 ? maxRaw : 0.0f;
    doc["peakToPeakRaw"] = sampleCount > 0 ? (static_cast<double>(maxRaw) - static_cast<double>(minRaw)) : 0.0;
    doc["maxAbsDeltaRaw"] = maxAbsDelta;
    doc["meanDeltaRaw"] = meanDelta;
    doc["deltaRmsRaw"] = deltaRms;
    doc["gapCount"] = 0;
    doc["maxGapUs"] = 0;
    doc["zeroDeltaCount"] = zeroDeltaCount;
    doc["driftPerMinRaw"] = 0.0;
    doc["qualityScore"] = qualityScore;
    doc["qualityLabel"] = qualityLabel;
    doc["gapRate"] = 0.0;
    doc["spikeRateEstimate"] = 0.0;
    doc["glitchThreshold"] = max(6.0 * deltaRms, 4.0 * stddev);
    doc["glitchRateEstimate"] = 0.0;
    doc["approxSnrDb"] = approxSnrDb;
    doc["notes"] = "recovered report generated from CSV (partial metrics)";

    serializeJsonPretty(doc, report);
    report.close();
    return true;
}

void flushCaptureBufferLocked()
{
    if (!rawCapture.active || !rawCapture.file || rawCapture.buffer.isEmpty()) {
        return;
    }
    rawCapture.file.print(rawCapture.buffer);
    rawCapture.buffer = "";
}

void writeCaptureReportLocked()
{
    if (rawCapture.reportPath.isEmpty()) {
        return;
    }

    File report = LittleFS.open(rawCapture.reportPath, "w");
    if (!report) {
        return;
    }

    const double mean = (rawCapture.sampleCount > 0) ? (rawCapture.sum / static_cast<double>(rawCapture.sampleCount)) : 0.0;
    double variance = 0.0;
    if (rawCapture.sampleCount > 0) {
        variance = (rawCapture.sumSquares / static_cast<double>(rawCapture.sampleCount)) - (mean * mean);
        if (variance < 0.0) variance = 0.0;
    }
    const double stddev = sqrt(variance);
    const double peakToPeak = static_cast<double>(rawCapture.maxRaw) - static_cast<double>(rawCapture.minRaw);
    const double durationSec = (rawCapture.endUs > rawCapture.startUs)
        ? static_cast<double>(rawCapture.endUs - rawCapture.startUs) / 1000000.0
        : 0.0;
    const double sampleRate = (durationSec > 0.0) ? static_cast<double>(rawCapture.sampleCount) / durationSec : 0.0;
    const double approxSnrDb = (stddev > 0.0 && fabs(mean) > 0.0)
        ? 20.0 * log10(fabs(mean) / stddev)
        : 0.0;
    const double meanDelta = (rawCapture.deltaCount > 0) ? (rawCapture.deltaSum / static_cast<double>(rawCapture.deltaCount)) : 0.0;
    const double deltaRms = (rawCapture.deltaCount > 0)
        ? sqrt(rawCapture.deltaSumSquares / static_cast<double>(rawCapture.deltaCount))
        : 0.0;

    const double n = static_cast<double>(rawCapture.sampleCount);
    const double elapsedSlope = (n > 1.0)
        ? ((n * rawCapture.elapsedRawProductSum) - (rawCapture.elapsedSum * rawCapture.elapsedRawSum)) /
          ((n * rawCapture.elapsedSumSquares) - (rawCapture.elapsedSum * rawCapture.elapsedSum) + 1e-12)
        : 0.0;
    const double driftPerMin = elapsedSlope * 60.0;

    const uint64_t expectedPeriodUs = rawCapture.expectedPeriodUs;
    const double expectedPeriodSec = (expectedPeriodUs > 0) ? static_cast<double>(expectedPeriodUs) / 1000000.0 : 0.0;
    const double gapRate = (rawCapture.sampleCount > 0) ? (static_cast<double>(rawCapture.gapCount) / n) : 0.0;
    const double spikeRate = (rawCapture.deltaCount > 0) ? (static_cast<double>(rawCapture.zeroDeltaCount) / static_cast<double>(rawCapture.deltaCount)) : 0.0;

    const double relativeNoise = (fabs(mean) > 1e-9) ? (stddev / fabs(mean)) : stddev;
    const double driftRatio = (fabs(mean) > 1e-9) ? (fabs(driftPerMin) / fabs(mean)) : fabs(driftPerMin);

    double qualityScore = 100.0;
    qualityScore -= min(30.0, gapRate * 4000.0);
    qualityScore -= min(35.0, spikeRate * 5000.0);
    qualityScore -= min(20.0, relativeNoise * 80.0);
    qualityScore -= min(15.0, driftRatio * 50.0);
    if (qualityScore < 0.0) qualityScore = 0.0;

    String qualityLabel = "excellent";
    if (qualityScore < 25.0) {
        qualityLabel = "unusable";
    } else if (qualityScore < 50.0) {
        qualityLabel = "poor";
    } else if (qualityScore < 75.0) {
        qualityLabel = "fair";
    } else if (qualityScore < 90.0) {
        qualityLabel = "good";
    }

    const double glitchThreshold = max(6.0 * deltaRms, 4.0 * stddev);
    const double glitchRateEstimate = (rawCapture.deltaCount > 0)
        ? (rawCapture.maxAbsDelta > glitchThreshold ? 1.0 : 0.0)
        : 0.0;

    JsonDocument doc;
    doc["file"] = rawCapture.filePath;
    doc["report"] = rawCapture.reportPath;
    doc["baselineKg"] = rawCapture.baselineKg;
    doc["testLoadKg"] = rawCapture.testLoadKg;
    doc["expectedSps"] = rawCapture.expectedSps;
    doc["expectedPeriodUs"] = expectedPeriodUs;
    doc["sampleCount"] = rawCapture.sampleCount;
    doc["durationMs"] = static_cast<uint32_t>((rawCapture.endUs > rawCapture.startUs) ? ((rawCapture.endUs - rawCapture.startUs) / 1000ULL) : 0ULL);
    doc["sampleRate"] = sampleRate;
    doc["meanRaw"] = mean;
    doc["stddevRaw"] = stddev;
    doc["minRaw"] = rawCapture.minRaw;
    doc["maxRaw"] = rawCapture.maxRaw;
    doc["peakToPeakRaw"] = peakToPeak;
    doc["maxAbsDeltaRaw"] = rawCapture.maxAbsDelta;
    doc["meanDeltaRaw"] = meanDelta;
    doc["deltaRmsRaw"] = deltaRms;
    doc["gapCount"] = rawCapture.gapCount;
    doc["maxGapUs"] = static_cast<uint64_t>(rawCapture.maxGapUs);
    doc["zeroDeltaCount"] = rawCapture.zeroDeltaCount;
    doc["driftPerMinRaw"] = driftPerMin;
    doc["qualityScore"] = qualityScore;
    doc["qualityLabel"] = qualityLabel;
    doc["gapRate"] = gapRate;
    doc["spikeRateEstimate"] = spikeRate;
    doc["glitchThreshold"] = glitchThreshold;
    doc["glitchRateEstimate"] = glitchRateEstimate;
    doc["approxSnrDb"] = approxSnrDb;
    doc["notes"] = "raw capture inclusive of glitches and dropouts";

    serializeJsonPretty(doc, report);
    report.close();
}

void finalizeRawCaptureLocked()
{
    if (!rawCapture.active) {
        return;
    }

    flushCaptureBufferLocked();
    if (rawCapture.file) {
        rawCapture.file.close();
    }

    writeCaptureReportLocked();

    rawCapture.lastFinalizeUs = static_cast<uint64_t>(esp_timer_get_time());
    rawCapture.lastFilePath = rawCapture.filePath;
    rawCapture.active = false;
    rawCapture.buffer = "";
}

void finalizeRawCapture(const char* reason, uint64_t finalizeUs)
{
    String pendingBuffer;
    String filePath;
    uint64_t startUs = 0;
    uint64_t endUs = 0;
    uint32_t sampleCount = 0;
    String stopReason;

    portENTER_CRITICAL(&rawCaptureMux);
    if (!rawCapture.active) {
        portEXIT_CRITICAL(&rawCaptureMux);
        return;
    }

    rawCapture.lastStopReason = (reason != nullptr) ? String(reason) : String("finalized");
    stopReason = rawCapture.lastStopReason;
    rawCapture.lastFinalizeUs = finalizeUs;
    rawCapture.lastFilePath = rawCapture.filePath;
    filePath = rawCapture.filePath;
    startUs = rawCapture.startUs;
    endUs = rawCapture.endUs;
    sampleCount = rawCapture.sampleCount;

    pendingBuffer = rawCapture.buffer;
    rawCapture.buffer = "";
    rawCapture.file = File();
    rawCapture.active = false;
    portEXIT_CRITICAL(&rawCaptureMux);

    if (!pendingBuffer.isEmpty() && !filePath.isEmpty()) {
        File appendFile = LittleFS.open(filePath, "a");
        if (appendFile) {
            appendFile.print(pendingBuffer);
            appendFile.close();
        }
    }

    writeCaptureReportLocked();

    logRawCaptureEvent("finalize", filePath, startUs, endUs, finalizeUs, sampleCount, stopReason);
}

void flushFsLiveProbeBufferLocked()
{
    if (!fsLiveProbe.active || !fsLiveProbe.file || fsLiveProbe.buffer.isEmpty()) {
        return;
    }
    fsLiveProbe.file.print(fsLiveProbe.buffer);
    fsLiveProbe.buffer = "";
}

void finalizeFsLiveProbe(const char* reason, uint64_t finalizeUs)
{
    File fileToClose;
    String pendingBuffer;
    String finalizedFilePath;

    portENTER_CRITICAL(&fsLiveProbeMux);
    if (!fsLiveProbe.active) {
        portEXIT_CRITICAL(&fsLiveProbeMux);
        return;
    }

    fsLiveProbe.lastStopReason = (reason != nullptr) ? String(reason) : String("finalized");
    fsLiveProbe.lastFinalizeUs = finalizeUs;
    fsLiveProbe.lastFilePath = fsLiveProbe.filePath;
    finalizedFilePath = fsLiveProbe.filePath;
    pendingBuffer = fsLiveProbe.buffer;
    fsLiveProbe.buffer = "";
    fileToClose = fsLiveProbe.file;
    fsLiveProbe.file = File();
    fsLiveProbe.active = false;
    portEXIT_CRITICAL(&fsLiveProbeMux);

    if (fileToClose) {
        if (!pendingBuffer.isEmpty()) {
            fileToClose.print(pendingBuffer);
        }
        fileToClose.close();
    }

    (void)finalizedFilePath;
}

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

std::vector<String> sortedCaptureFiles()
{
    std::vector<String> names;
    if (LittleFS.exists(CAPTURES_DIR)) {
        File dir = LittleFS.open(CAPTURES_DIR);
        if (dir && dir.isDirectory()) {
            File entry;
            while ((entry = dir.openNextFile())) {
                if (!entry.isDirectory()) {
                    String name = String(entry.name());
                    int slash = name.lastIndexOf('/');
                    if (slash >= 0) name = name.substring(slash + 1);
                    if (name.endsWith(".csv")) names.push_back(name);
                }
                entry.close();
            }
            dir.close();
        }
    }

    // Fallback: some LittleFS configurations are more reliable when files are
    // created directly in root, so include /raw_*.csv captures too.
    File root = LittleFS.open("/");
    if (root && root.isDirectory()) {
        File entry;
        while ((entry = root.openNextFile())) {
            if (!entry.isDirectory()) {
                String name = String(entry.name());
                int slash = name.lastIndexOf('/');
                if (slash >= 0) name = name.substring(slash + 1);
                if (name.startsWith("raw_") && name.endsWith(".csv")) {
                    bool exists = false;
                    for (const String& existing : names) {
                        if (existing == name) {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists) {
                        names.push_back(name);
                    }
                }
            }
            entry.close();
        }
        root.close();
    }

    std::sort(names.begin(), names.end());
    return names;
}

} // anonymous namespace


namespace MetaSense::RunStorage {

void save(const MetaSense::Telemetry& telemetry)
{
    portENTER_CRITICAL(&telemetryMux);
    lastRun = telemetry;
    ++telemetryVersion;
    portEXIT_CRITICAL(&telemetryMux);

    if (publishTaskHandle != nullptr) {
        xTaskNotifyGive(publishTaskHandle);
    }
}

MetaSense::Telemetry latest()
{
    MetaSense::Telemetry snapshot;
    portENTER_CRITICAL(&telemetryMux);
    snapshot = lastRun;
    portEXIT_CRITICAL(&telemetryMux);
    return snapshot;
}

uint32_t version()
{
    uint32_t currentVersion = 0;
    portENTER_CRITICAL(&telemetryMux);
    currentVersion = telemetryVersion;
    portEXIT_CRITICAL(&telemetryMux);
    return currentVersion;
}

void setPublishTaskHandle(TaskHandle_t handle)
{
    publishTaskHandle = handle;
}

void flush() {}

bool startRawCapture(uint32_t durationMs, float baselineKg, float testLoadKg, String& outFilePath)
{
    // Defensive normalization: keep capture window in a practical range.
    // This prevents accidental short windows from alternate call paths.
    if (durationMs < 10000U || durationMs > 300000U) {
        durationMs = 20000;
    }

    const bool haveCaptureDir = LittleFS.exists(CAPTURES_DIR) || LittleFS.mkdir(CAPTURES_DIR);
    const String baseDir = haveCaptureDir ? String(CAPTURES_DIR) : String("/");

    portENTER_CRITICAL(&rawCaptureMux);
    const bool unavailable = rawCapture.active || rawCaptureStartPending;
    if (!unavailable) {
        rawCaptureStartPending = true;
    }
    portEXIT_CRITICAL(&rawCaptureMux);
    if (unavailable) {
        return false;
    }

    const uint32_t epoch = getEpochTime();
    String base = makeCaptureBaseName(epoch);
    String csvPath = baseDir + (baseDir == "/" ? "" : "/") + base + ".csv";
    String reportPath = baseDir + (baseDir == "/" ? "" : "/") + base + ".json";

    int suffix = 1;
    while (LittleFS.exists(csvPath)) {
        csvPath = baseDir + (baseDir == "/" ? "" : "/") + base + "_" + String(suffix++) + ".csv";
        reportPath = csvPath;
        reportPath.replace(".csv", ".json");
        if (suffix > 1000) {
            portENTER_CRITICAL(&rawCaptureMux);
            rawCaptureStartPending = false;
            portEXIT_CRITICAL(&rawCaptureMux);
            return false;
        }
    }

    File headerFile = LittleFS.open(csvPath, "w");
    if (!headerFile) {
        portENTER_CRITICAL(&rawCaptureMux);
        rawCaptureStartPending = false;
        portEXIT_CRITICAL(&rawCaptureMux);
        return false;
    }

    const uint64_t startUs = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t endUs = startUs + (static_cast<uint64_t>(durationMs) * 1000ULL);
    const uint32_t startMs = millis();
    const uint32_t endMs = startMs + durationMs;
    const uint16_t expectedSps = MetaSense::Input::getLoadCellSampleRateSps();
    String header;
    header.reserve(256);
    header = "# MetaSense raw capture\n";
    header += "# baselineKg=" + String(baselineKg, 3) + "\n";
    header += "# testLoadKg=" + String(testLoadKg, 3) + "\n";
    header += "# durationMs=" + String(durationMs) + "\n";
    header += "timestamp_us,source,raw_value,filtered_value\n";

    headerFile.print(header);
    headerFile.close();

    portENTER_CRITICAL(&rawCaptureMux);
    rawCapture.file = File();
    rawCapture.filePath = csvPath;
    rawCapture.reportPath = reportPath;
    rawCapture.buffer.reserve(4096);
    rawCapture.buffer = "";
    rawCapture.startUs = startUs;
    rawCapture.endUs = endUs;
    rawCapture.startMs = startMs;
    rawCapture.endMs = endMs;
    rawCapture.baselineKg = baselineKg;
    rawCapture.testLoadKg = testLoadKg;
    rawCapture.sampleCount = 0;
    rawCapture.lastSampleUs = 0;
    rawCapture.sum = 0.0;
    rawCapture.sumSquares = 0.0;
    rawCapture.minRaw = 0.0f;
    rawCapture.maxRaw = 0.0f;
    rawCapture.havePrev = false;
    rawCapture.maxAbsDelta = 0.0f;
    rawCapture.deltaSum = 0.0;
    rawCapture.deltaSumSquares = 0.0;
    rawCapture.deltaCount = 0;
    rawCapture.expectedSps = expectedSps;
    rawCapture.expectedPeriodUs = (expectedSps > 0) ? (1000000ULL / static_cast<uint64_t>(expectedSps)) : 0ULL;
    rawCapture.gapCount = 0;
    rawCapture.maxGapUs = 0;
    rawCapture.zeroDeltaCount = 0;
    rawCapture.elapsedSum = 0.0;
    rawCapture.elapsedSumSquares = 0.0;
    rawCapture.elapsedRawSum = 0.0;
    rawCapture.elapsedRawProductSum = 0.0;
    rawCapture.lastDurationMs = durationMs;
    rawCapture.lastStopReason = "active";
    rawCapture.active = true;
    rawCaptureStartPending = false;
    portEXIT_CRITICAL(&rawCaptureMux);

    logRawCaptureEvent("start", csvPath, startUs, endUs, static_cast<uint64_t>(esp_timer_get_time()), 0, "active");

    outFilePath = csvPath;
    return true;
}

void appendRawCaptureSample(uint64_t timestampUs, float rawValue, float filteredValue, const char* sourceLabel)
{
    bool shouldFinalize = false;
    String targetPath;
    char line[128];
    portENTER_CRITICAL(&rawCaptureMux);
    if (!rawCapture.active) {
        portEXIT_CRITICAL(&rawCaptureMux);
        return;
    }

    if (rawCapture.sampleCount == 0) {
        rawCapture.minRaw = rawValue;
        rawCapture.maxRaw = rawValue;
    } else {
        const uint64_t elapsedUs = (timestampUs > rawCapture.startUs) ? (timestampUs - rawCapture.startUs) : 0ULL;
        if (rawCapture.expectedPeriodUs > 0 && elapsedUs > rawCapture.expectedPeriodUs * 2ULL) {
            const uint64_t gapUs = elapsedUs - rawCapture.expectedPeriodUs;
            rawCapture.gapCount++;
            if (gapUs > rawCapture.maxGapUs) rawCapture.maxGapUs = gapUs;
        }
        if (rawValue < rawCapture.minRaw) rawCapture.minRaw = rawValue;
        if (rawValue > rawCapture.maxRaw) rawCapture.maxRaw = rawValue;
        const float delta = fabsf(rawValue - rawCapture.prevRaw);
        if (delta == 0.0f) {
            rawCapture.zeroDeltaCount++;
        }
        rawCapture.deltaSum += delta;
        rawCapture.deltaSumSquares += static_cast<double>(delta) * static_cast<double>(delta);
        rawCapture.deltaCount++;
        if (delta > rawCapture.maxAbsDelta) rawCapture.maxAbsDelta = delta;
    }

    rawCapture.prevRaw = rawValue;
    rawCapture.havePrev = true;
    rawCapture.sampleCount++;
    rawCapture.lastSampleUs = timestampUs;
    rawCapture.sum += static_cast<double>(rawValue);
    rawCapture.sumSquares += static_cast<double>(rawValue) * static_cast<double>(rawValue);

    const double elapsedSec = (timestampUs > rawCapture.startUs)
        ? static_cast<double>(timestampUs - rawCapture.startUs) / 1000000.0
        : 0.0;
    rawCapture.elapsedSum += elapsedSec;
    rawCapture.elapsedSumSquares += elapsedSec * elapsedSec;
    rawCapture.elapsedRawSum += static_cast<double>(rawValue);
    rawCapture.elapsedRawProductSum += elapsedSec * static_cast<double>(rawValue);

    targetPath = rawCapture.filePath;

    snprintf(line, sizeof(line), "%llu,%s,%.6f,%.6f\n",
             static_cast<unsigned long long>(timestampUs),
             (sourceLabel != nullptr) ? sourceLabel : "unknown",
             static_cast<double>(rawValue),
             static_cast<double>(filteredValue));

    const uint32_t elapsedMs = millis() - rawCapture.startMs;
    if (elapsedMs >= rawCapture.lastDurationMs) {
        shouldFinalize = true;
    }

    portEXIT_CRITICAL(&rawCaptureMux);

    if (!targetPath.isEmpty()) {
        File appendFile = LittleFS.open(targetPath, "a");
        if (appendFile) {
            appendFile.print(line);
            appendFile.close();
        }
    }

    if (shouldFinalize) {
        finalizeRawCapture("timeout_append", timestampUs);
    }
}

void tickRawCapture(uint64_t nowUs)
{
    bool shouldFinalize = false;
    portENTER_CRITICAL(&rawCaptureMux);
    if (rawCapture.active) {
        const uint32_t elapsedMs = millis() - rawCapture.startMs;
        if (elapsedMs >= rawCapture.lastDurationMs) {
            shouldFinalize = true;
        }
    }
    portEXIT_CRITICAL(&rawCaptureMux);

    if (shouldFinalize) {
        finalizeRawCapture("timeout_tick", nowUs);
    }
}

bool rawCaptureActive()
{
    portENTER_CRITICAL(&rawCaptureMux);
    const bool active = rawCapture.active;
    portEXIT_CRITICAL(&rawCaptureMux);
    return active;
}

String rawCaptureStateJson()
{
    JsonDocument doc;
    portENTER_CRITICAL(&rawCaptureMux);
    const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
    doc["active"] = rawCapture.active;
    doc["file"] = rawCapture.filePath;
    doc["sampleCount"] = rawCapture.sampleCount;
    doc["startUs"] = rawCapture.startUs;
    doc["endUs"] = rawCapture.endUs;
    doc["nowUs"] = nowUs;
    doc["lastSampleUs"] = rawCapture.lastSampleUs;
    doc["lastStopReason"] = rawCapture.lastStopReason;
    doc["lastFinalizeUs"] = rawCapture.lastFinalizeUs;
    doc["lastDurationMs"] = rawCapture.lastDurationMs;
    doc["lastFile"] = rawCapture.lastFilePath;
    const uint32_t elapsedMs = (rawCapture.startUs > 0 && nowUs >= rawCapture.startUs)
        ? static_cast<uint32_t>((nowUs - rawCapture.startUs) / 1000ULL)
        : 0U;
    const uint32_t remainingMs = (rawCapture.active && rawCapture.endUs > nowUs)
        ? static_cast<uint32_t>((rawCapture.endUs - nowUs) / 1000ULL)
        : 0U;
    doc["elapsedMs"] = elapsedMs;
    doc["remainingMs"] = remainingMs;
    portEXIT_CRITICAL(&rawCaptureMux);

    String out;
    serializeJson(doc, out);
    return out;
}

String loadRawCaptureReport(const String& filename)
{
    if (filename.indexOf('/') >= 0 || filename.indexOf("..") >= 0) return "{}";

    String base = filename;
    if (base.endsWith(".csv")) {
        base.remove(base.length() - 4);
    }

    String reportPath = captureReportPathForBase(base);
    File report = LittleFS.open(reportPath, "r");
    if (!report && filename.endsWith(".csv")) {
        bool activeMatches = false;
        portENTER_CRITICAL(&rawCaptureMux);
        activeMatches = rawCapture.active && rawCapture.filePath.endsWith(filename);
        portEXIT_CRITICAL(&rawCaptureMux);
        if (activeMatches) {
            return rawCaptureStateJson();
        }
        (void)trySynthesizeCaptureReportFromCsv(filename);
        report = LittleFS.open(reportPath, "r");
    }
    if (!report) return "{}";

    String out = report.readString();
    report.close();
    return out.length() > 0 ? out : "{}";
}

String verifyRawCaptureCsv(const String& filename)
{
    String selected = filename;
    if (selected.isEmpty()) {
        auto names = sortedCaptureFiles();
        if (names.empty()) {
            return "{\"ok\":false,\"msg\":\"no captures\"}";
        }
        selected = names.back();
    }

    if (!selected.endsWith(".csv")) {
        selected += ".csv";
    }
    if (selected.indexOf('/') >= 0 || selected.indexOf("..") >= 0) {
        return "{\"ok\":false,\"msg\":\"invalid filename\"}";
    }

    const String path = captureCsvPathForName(selected);
    if (path.isEmpty()) {
        return "{\"ok\":false,\"msg\":\"capture not found\"}";
    }

    File file = LittleFS.open(path, "r");
    if (!file) {
        return "{\"ok\":false,\"msg\":\"open failed\"}";
    }

    bool headerSeen = false;
    uint32_t sampleLines = 0;
    uint32_t expectedDurationMs = 0;
    uint64_t firstTsUs = 0;
    uint64_t lastTsUs = 0;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) {
            continue;
        }

        if (line.startsWith("# durationMs=")) {
            expectedDurationMs = static_cast<uint32_t>(line.substring(13).toInt());
            continue;
        }

        if (line.startsWith("#")) {
            continue;
        }

        if (!headerSeen) {
            headerSeen = (line == "timestamp_us,source,raw_value" ||
                          line == "timestamp_us,source,raw_value,filtered_value");
            continue;
        }

        const int comma1 = line.indexOf(',');
        const int comma2 = (comma1 >= 0) ? line.indexOf(',', comma1 + 1) : -1;
        if (comma1 <= 0 || comma2 <= comma1) {
            continue;
        }

        const uint64_t tsUs = static_cast<uint64_t>(strtoull(line.substring(0, comma1).c_str(), nullptr, 10));
        if (sampleLines == 0) {
            firstTsUs = tsUs;
        }
        lastTsUs = tsUs;
        sampleLines++;
    }

    file.close();

    const uint32_t spanMs = (sampleLines > 1 && lastTsUs >= firstTsUs)
        ? static_cast<uint32_t>((lastTsUs - firstTsUs) / 1000ULL)
        : 0U;

    uint32_t targetMs = expectedDurationMs;
    if (targetMs == 0U) {
        targetMs = 20000U;
    }

    const bool durationOk = spanMs >= static_cast<uint32_t>((targetMs * 95ULL) / 100ULL);
    const bool complete = headerSeen && sampleLines > 0 && durationOk;

    String out = "{\"ok\":true";
    out += ",\"file\":\"" + selected + "\"";
    out += ",\"path\":\"" + path + "\"";
    out += ",\"headerSeen\":" + String(headerSeen ? "true" : "false");
    out += ",\"sampleLines\":" + String(sampleLines);
    out += ",\"expectedDurationMs\":" + String(targetMs);
    out += ",\"spanMs\":" + String(spanMs);
    out += ",\"complete\":" + String(complete ? "true" : "false");
    out += ",\"msg\":\"" + String(complete ? "complete" : "incomplete") + "\"}";
    return out;
}

String listRawCaptures()
{
    auto names = sortedCaptureFiles();
    String activeFilename;
    portENTER_CRITICAL(&rawCaptureMux);
    if (rawCapture.active) {
        activeFilename = rawCapture.filePath;
        int slash = activeFilename.lastIndexOf('/');
        if (slash >= 0) {
            activeFilename = activeFilename.substring(slash + 1);
        }
    }
    portEXIT_CRITICAL(&rawCaptureMux);

    String out = "[";
    bool first = true;

    for (const String& fname : names) {
        String base = fname;
        if (base.endsWith(".csv")) base.remove(base.length() - 4);
        String reportPath = captureReportPathForBase(base);

        JsonDocument meta;
        if (!LittleFS.exists(reportPath) && !(rawCaptureActive() && activeFilename == fname)) {
            (void)trySynthesizeCaptureReportFromCsv(fname);
        }
        File report = LittleFS.open(reportPath, "r");
        if (report) {
            deserializeJson(meta, report);
            report.close();
        }

        if (!first) out += ",";
        first = false;

        out += "{\"filename\":\"" + fname + "\"";
        out += ",\"report\":\"" + base + ".json\"";
        out += ",\"sampleCount\":" + String(meta["sampleCount"] | 0);
        out += ",\"durationMs\":" + String(meta["durationMs"] | 0);
        out += ",\"sampleRate\":" + String(meta["sampleRate"] | 0.0, 1);
        out += ",\"qualityScore\":" + String(meta["qualityScore"] | 0.0, 1);
        out += ",\"qualityLabel\":\"" + String((const char*)(meta["qualityLabel"] | "unknown")) + "\"";
        out += ",\"gapCount\":" + String(meta["gapCount"] | 0);
        out += ",\"maxGapUs\":" + String((unsigned long)(meta["maxGapUs"] | 0ULL));
        out += ",\"driftPerMinRaw\":" + String(meta["driftPerMinRaw"] | 0.0, 6);
        out += ",\"approxSnrDb\":" + String(meta["approxSnrDb"] | 0.0, 2);
        out += ",\"testLoadKg\":" + String(meta["testLoadKg"] | 0.0, 3);
        out += ",\"baselineKg\":" + String(meta["baselineKg"] | 0.0, 3);
        out += "}";
    }

    out += "]";
    return out;
}

bool startFsLiveProbe(uint32_t durationMs, String& outFilePath)
{
    if (durationMs < 1000U || durationMs > 120000U) {
        durationMs = 10000U;
    }

    const bool haveCaptureDir = LittleFS.exists(CAPTURES_DIR) || LittleFS.mkdir(CAPTURES_DIR);
    const String baseDir = haveCaptureDir ? String(CAPTURES_DIR) : String("/");

    portENTER_CRITICAL(&fsLiveProbeMux);
    const bool alreadyActive = fsLiveProbe.active;
    portEXIT_CRITICAL(&fsLiveProbeMux);
    if (alreadyActive) {
        return false;
    }

    const uint32_t epoch = getEpochTime();
    String base = makeProbeBaseName(epoch);
    String csvPath = baseDir + (baseDir == "/" ? "" : "/") + base + ".csv";
    int suffix = 1;
    while (LittleFS.exists(csvPath)) {
        csvPath = baseDir + (baseDir == "/" ? "" : "/") + base + "_" + String(suffix++) + ".csv";
        if (suffix > 1000) {
            return false;
        }
    }

    File probeFile = LittleFS.open(csvPath, "w");
    if (!probeFile) {
        return false;
    }

    const uint64_t startUs = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t endUs = startUs + (static_cast<uint64_t>(durationMs) * 1000ULL);
    String header;
    header.reserve(128);
    header = "timestamp_us,value\n";
    probeFile.print(header);

    portENTER_CRITICAL(&fsLiveProbeMux);
    fsLiveProbe.file = probeFile;
    fsLiveProbe.filePath = csvPath;
    fsLiveProbe.buffer.reserve(2048);
    fsLiveProbe.buffer = "";
    fsLiveProbe.startUs = startUs;
    fsLiveProbe.endUs = endUs;
    fsLiveProbe.sampleCount = 0;
    fsLiveProbe.lastSampleUs = 0;
    fsLiveProbe.lastDurationMs = durationMs;
    fsLiveProbe.lastStopReason = "active";
    fsLiveProbe.active = true;
    portEXIT_CRITICAL(&fsLiveProbeMux);

    outFilePath = csvPath;
    return true;
}

void appendFsLiveProbeSample(uint64_t timestampUs, float value)
{
    bool shouldFinalize = false;
    portENTER_CRITICAL(&fsLiveProbeMux);
    if (!fsLiveProbe.active) {
        portEXIT_CRITICAL(&fsLiveProbeMux);
        return;
    }

    fsLiveProbe.sampleCount++;
    fsLiveProbe.lastSampleUs = timestampUs;

    char line[64];
    snprintf(line, sizeof(line), "%llu,%.6f\n",
             static_cast<unsigned long long>(timestampUs),
             static_cast<double>(value));
    fsLiveProbe.buffer += line;

    if (timestampUs >= fsLiveProbe.endUs) {
        shouldFinalize = true;
    }

    portEXIT_CRITICAL(&fsLiveProbeMux);

    if (shouldFinalize) {
        finalizeFsLiveProbe("timeout_append", timestampUs);
    }
}

void tickFsLiveProbe(uint64_t nowUs)
{
    bool shouldFinalize = false;
    portENTER_CRITICAL(&fsLiveProbeMux);
    if (fsLiveProbe.active && nowUs >= fsLiveProbe.endUs) {
        shouldFinalize = true;
    }
    portEXIT_CRITICAL(&fsLiveProbeMux);

    if (shouldFinalize) {
        finalizeFsLiveProbe("timeout_tick", nowUs);
    }
}

String fsLiveProbeStateJson()
{
    JsonDocument doc;
    portENTER_CRITICAL(&fsLiveProbeMux);
    const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
    doc["active"] = fsLiveProbe.active;
    doc["file"] = fsLiveProbe.filePath;
    doc["sampleCount"] = fsLiveProbe.sampleCount;
    doc["startUs"] = fsLiveProbe.startUs;
    doc["endUs"] = fsLiveProbe.endUs;
    doc["nowUs"] = nowUs;
    doc["lastSampleUs"] = fsLiveProbe.lastSampleUs;
    doc["lastStopReason"] = fsLiveProbe.lastStopReason;
    doc["lastFinalizeUs"] = fsLiveProbe.lastFinalizeUs;
    doc["lastDurationMs"] = fsLiveProbe.lastDurationMs;
    doc["lastFile"] = fsLiveProbe.lastFilePath;
    const uint32_t elapsedMs = (fsLiveProbe.startUs > 0 && nowUs >= fsLiveProbe.startUs)
        ? static_cast<uint32_t>((nowUs - fsLiveProbe.startUs) / 1000ULL)
        : 0U;
    const uint32_t remainingMs = (fsLiveProbe.active && fsLiveProbe.endUs > nowUs)
        ? static_cast<uint32_t>((fsLiveProbe.endUs - nowUs) / 1000ULL)
        : 0U;
    doc["elapsedMs"] = elapsedMs;
    doc["remainingMs"] = remainingMs;
    portEXIT_CRITICAL(&fsLiveProbeMux);

    String out;
    serializeJson(doc, out);
    return out;
}

String fsLiveProbeReadText()
{
    String path;
    portENTER_CRITICAL(&fsLiveProbeMux);
    path = fsLiveProbe.active ? fsLiveProbe.filePath : fsLiveProbe.lastFilePath;
    if (path.isEmpty()) {
        path = fsLiveProbe.filePath;
    }
    portEXIT_CRITICAL(&fsLiveProbeMux);

    if (path.isEmpty()) {
        return "";
    }

    File file = LittleFS.open(path, "r");
    if (!file) {
        return "";
    }
    const String text = file.readString();
    file.close();
    return text;
}

String fsLiveProbeVerifyJson()
{
    String path;
    uint32_t expectedMs = 60000U;
    portENTER_CRITICAL(&fsLiveProbeMux);
    path = fsLiveProbe.active ? fsLiveProbe.filePath : fsLiveProbe.lastFilePath;
    if (path.isEmpty()) {
        path = fsLiveProbe.filePath;
    }
    if (fsLiveProbe.lastDurationMs > 0U) {
        expectedMs = fsLiveProbe.lastDurationMs;
    }
    portEXIT_CRITICAL(&fsLiveProbeMux);

    if (path.isEmpty()) {
        return "{\"ok\":false,\"msg\":\"no probe file\"}";
    }

    File file = LittleFS.open(path, "r");
    if (!file) {
        return "{\"ok\":false,\"msg\":\"open failed\"}";
    }

    bool headerSeen = false;
    uint32_t sampleLines = 0;
    uint64_t firstTsUs = 0;
    uint64_t lastTsUs = 0;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) {
            continue;
        }

        if (!headerSeen) {
            headerSeen = (line == "timestamp_us,value");
            continue;
        }

        const int comma = line.indexOf(',');
        if (comma <= 0) {
            continue;
        }
        const uint64_t tsUs = static_cast<uint64_t>(strtoull(line.substring(0, comma).c_str(), nullptr, 10));
        if (sampleLines == 0) {
            firstTsUs = tsUs;
        }
        lastTsUs = tsUs;
        sampleLines++;
    }
    file.close();

    const uint32_t spanMs = (sampleLines > 1 && lastTsUs >= firstTsUs)
        ? static_cast<uint32_t>((lastTsUs - firstTsUs) / 1000ULL)
        : 0U;
    const bool durationOk = spanMs >= static_cast<uint32_t>((expectedMs * 95ULL) / 100ULL);
    const bool complete = headerSeen && sampleLines > 0 && durationOk;

    String out = "{\"ok\":true";
    out += ",\"path\":\"" + path + "\"";
    out += ",\"headerSeen\":" + String(headerSeen ? "true" : "false");
    out += ",\"sampleLines\":" + String(sampleLines);
    out += ",\"expectedDurationMs\":" + String(expectedMs);
    out += ",\"spanMs\":" + String(spanMs);
    out += ",\"complete\":" + String(complete ? "true" : "false");
    out += ",\"msg\":\"" + String(complete ? "complete" : "incomplete") + "\"}";
    return out;
}

void saveCalibration()
{
    Preferences prefs;
    if (!prefs.begin("calib", false)) {
        return;
    }

    prefs.putBool("init", true);
    prefs.putFloat("zero", MetaSense::Input::getZeroOffset());
    prefs.putFloat("factor", MetaSense::Input::getCalibrationFactor());
    prefs.end();
}

bool loadCalibration(float& zeroOffset, float& calibrationFactor)
{
    Preferences prefs;
    if (!prefs.begin("calib", true)) {
        return false;
    }

    const bool hasData = prefs.getBool("init", false);
    if (!hasData) {
        prefs.end();
        return false;
    }

    zeroOffset = prefs.getFloat("zero", 0.0f);
    calibrationFactor = prefs.getFloat("factor", 0.01f);
    prefs.end();
    return true;
}

// ─── Persistent run history ─────────────────────────────────────────────────

bool saveRun(const String& payload)
{
    const uint32_t payloadHash = fnv1a32(payload);
    const uint32_t nowMs = millis();
    if (lastRunPayloadHash == payloadHash && (nowMs - lastRunPayloadMs) <= kRunSaveDedupWindowMs) {
        return true;
    }

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

    lastRunPayloadHash = payloadHash;
    lastRunPayloadMs = nowMs;
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

