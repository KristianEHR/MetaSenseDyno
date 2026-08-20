#include "Notify.h"
#include "Telemetry.h"
#include "YourWebsocketObject.h"   // replace with your actual ws include

void notifyClients(const Telemetry &data, bool isRecording)
{
    String json;
    json.reserve(768);

    json = "{";
    json += "\"type\":\"data\",";
    json += "\"rpm\":" + String(data.rpm, 0) + ",";
    json += "\"hp\":" + String(data.hp, 2) + ",";
    json += "\"torque\":" + String(data.torque, 2) + ",";
    json += "\"brakeTorque\":" + String(data.brakeTorque, 2) + ",";

    json += "\"leaf\":{";
    json += "\"rpm\":" + String(data.leaf_rpm, 0) + ",";
    json += "\"torque\":" + String(data.leaf_torqueNm, 2) + ",";
    json += "\"inv_temp\":" + String(data.leaf_invTempC, 1) + ",";
    json += "\"motor_temp\":" + String(data.leaf_motorTempC, 1) + ",";
    json += "\"coolant_temp\":" + String(data.leaf_coolantTempC, 1) + ",";
    json += "\"inv_ready\":" + String(data.leaf_invReady ? "true" : "false") + ",";
    json += "\"inv_fault\":" + String(data.leaf_invFault ? "true" : "false") + ",";
    json += "\"hv_interlock_ok\":" + String(data.leaf_hvInterlockOk ? "true" : "false") + ",";
    json += "\"gear_state\":" + String(data.leaf_gearState);
    json += "},";

    json += "\"recording\":" + String(isRecording ? "true" : "false");
    json += "}";

    ws.textAll(json);
}
