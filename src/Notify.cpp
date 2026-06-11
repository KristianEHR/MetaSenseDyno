#include "Notify.h"

namespace MetaSense::Notify {

void sendInfo(const String& msg)
{
    Serial.println("[INFO] " + msg);
}

void sendError(const String& msg)
{
    Serial.println("[ERROR] " + msg);
}

void sendStatus(const String& msg)
{
    Serial.println("[STATUS] " + msg);
}

} // namespace MetaSense::Notify
