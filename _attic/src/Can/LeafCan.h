#pragma once
#include <driver/twai.h>
#include "../Telemetry/Telemetry.h"

namespace LeafCan {

struct LeafCanData {
    float motorRpm       = 0.0f;
    float torqueNm       = 0.0f;
    float inverterTempC  = 0.0f;
    float motorTempC     = 0.0f;
    float coolantTempC   = 0.0f;
    bool  ready          = false;
    bool  fault          = false;
    bool  hvInterlock    = false;
    uint8_t gearState    = 0;
};

bool begin();
bool decode(const twai_message_t &msg, MetaSense::Telemetry &out);
void poll(MetaSense::Telemetry &out);

const LeafCanData &data();

} // namespace LeafCan
