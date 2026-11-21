#pragma once
#include <ESPAsyncWebServer.h>
#include "Telemetry.h"

namespace MetaSense {

    extern AsyncWebSocket ws;     // declared only
    extern Telemetry live;        // declared only
    extern bool isRecording;      // declared only

}
