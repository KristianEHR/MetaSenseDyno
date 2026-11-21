#pragma once
#include <Arduino.h>

namespace MetaSense::Notify {

    void sendInfo(const String& msg);
    void sendError(const String& msg);
    void sendStatus(const String& msg);

}
