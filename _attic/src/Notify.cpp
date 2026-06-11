#include "Notify.h"
#include "../Globals.h"
#include "../Web/WebSocket.h"

namespace MetaSense::Notify {

void send(const Telemetry &data, bool isRecording)
{
    String json;
    json.reserve(900);

    json = "{";
    json += "\"rpm\":" + String(data.rpm) + ",";
    json += "\"torque\":" + String(data.torque) + ",";
    json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";

    json += "\"leaf\":{";
    json += "\"rpm\":" + String(data.leaf_rpm) + ",";
    json += "\"torque\":" + String(data.leaf_torqueNm);
    json += "}";
    json += "}";

    // Use the new WebSocket broadcast API
    MetaSense::WebSocket::broadcast(json);
}

} // namespace MetaSense::Notify
