#include "Notify.h"
#include "WebSocket.h"
#include <ESPAsyncWebServer.h>

namespace {   // LOCAL SCOPE

void sendJson(const String& type, const String& msg)
{
    String json;
    json.reserve(256);

    json = "{";
    json += "\"type\":\"" + type + "\",";
    json += "\"msg\":\"" + msg + "\"";
    json += "}";

    MetaSense::WebSocket::socket().textAll(json);
}

} // anonymous namespace


namespace MetaSense::Notify {   // EXTERNAL SCOPE

void sendInfo(const String& msg)
{
    sendJson("info", msg);
}

void sendError(const String& msg)
{
    sendJson("error", msg);
}

void sendStatus(const String& msg)
{
    sendJson("status", msg);
}

} // namespace MetaSense::Notify
