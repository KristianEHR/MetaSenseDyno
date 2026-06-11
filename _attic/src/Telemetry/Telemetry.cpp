#include "Telemetry.h"
#include "Globals.h"

namespace MetaSense {

void Telemetry::init() {}

void Telemetry::update() {
    if (isRecording) {
        // write CSV, store samples
    }
}

} // namespace MetaSense
