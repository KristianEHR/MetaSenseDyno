#include "LeafCan.h"

namespace LeafCan {

static LeafCanData g_data{};

bool begin() {
    return true;
}

bool decode(const twai_message_t &msg, MetaSense::Telemetry &out) {
    switch (msg.identifier) {
        case 0x1DA:
            g_data.motorRpm = 0.0f;
            break;
        case 0x1DB:
            g_data.torqueNm = 0.0f;
            break;
        case 0x1DC:
            g_data.inverterTempC = 0.0f;
            g_data.motorTempC    = 0.0f;
            g_data.coolantTempC  = 0.0f;
            break;
        case 0x1D4:
            g_data.ready       = true;
            g_data.fault       = false;
            g_data.hvInterlock = true;
            g_data.gearState   = 0;
            break;
        default:
            return false;
    }

    out.leaf_rpm          = g_data.motorRpm;
    out.leaf_torqueNm     = g_data.torqueNm;
    out.leaf_invTempC     = g_data.inverterTempC;
    out.leaf_motorTempC   = g_data.motorTempC;
    out.leaf_coolantTempC = g_data.coolantTempC;
    out.leaf_invReady     = g_data.ready;
    out.leaf_invFault     = g_data.fault;
    out.leaf_hvInterlockOk= g_data.hvInterlock;
    out.leaf_gearState    = g_data.gearState;

    return true;
}

void poll(MetaSense::Telemetry &out) {
    out.leaf_rpm          = g_data.motorRpm;
    out.leaf_torqueNm     = g_data.torqueNm;
    out.leaf_invTempC     = g_data.inverterTempC;
    out.leaf_motorTempC   = g_data.motorTempC;
    out.leaf_coolantTempC = g_data.coolantTempC;
    out.leaf_invReady     = g_data.ready;
    out.leaf_invFault     = g_data.fault;
    out.leaf_hvInterlockOk= g_data.hvInterlock;
    out.leaf_gearState    = g_data.gearState;
}

const LeafCanData &data() {
    return g_data;
}

} // namespace LeafCan
