#include "TelemetryPublisher.h"
#include "MetaSense.h"
#include "DynoState.h"

namespace {

    // 20 Hz telemetry by default
    constexpr uint32_t TELEMETRY_PERIOD_MS = 50;
    uint32_t lastPublishMs = 0;

} // anonymous namespace

namespace TelemetryPublisher {

    void begin() {
        lastPublishMs = millis();
    }

    String buildJson(const Telemetry &t, bool isRecording) {
        // Keep it manual to avoid extra deps; adapt fields to your Telemetry struct
        String json = "{";

        json += "\"type\":\"telemetry\"";
        json += ",\"isRecording\":"; json += (isRecording ? "true" : "false");

        // Example fields – replace with your real Telemetry members
        json += ",\"rpm\":";          json += String(t.rpm, 0);
        json += ",\"torque\":";       json += String(t.torque, 2);
        json += ",\"dcBusVoltage\":"; json += String(t.dcBusVoltage, 2);
        json += ",\"invTemp\":";      json += String(t.inverterTemp, 1);

        // Dyno mode from state machine
        json += ",\"dynoMode\":\"";   json += DynoState::modeName(); json += "\"";

        json += "}";

        return json;
    }

    void loop() {
        uint32_t now = millis();
        if (now - lastPublishMs < TELEMETRY_PERIOD_MS) {
            return;
        }
        lastPublishMs = now;

        String json = buildJson(MetaSense::live, MetaSense::isRecording);
        MetaSense::ws.textAll(json);
    }

} // namespace TelemetryPublisher
