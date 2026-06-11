#pragma once
#include "Telemetry/Telemetry.h"

namespace MetaSense::Notify {
    void send(const Telemetry &data, bool isRecording);
}
