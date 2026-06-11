#pragma once
#include <ESPAsyncWebServer.h>
#include "Telemetry.h"

namespace MetaSense {

    extern AsyncWebSocket ws;     // declared only
    extern Telemetry live;        // declared only
    extern bool isRecording;      // declared only

}
// MetaSense.h (you showed this earlier)
namespace MetaSense {
    extern Telemetry live;
    void setup();
    void loopOnce();
    void notifyClients(const Telemetry &data, bool isRecording);
    extern AsyncWebSocket ws;
    extern bool isRecording;
}
