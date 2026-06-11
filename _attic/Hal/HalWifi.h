#pragma once
#include <Arduino.h>

namespace HalWifi {

    void setClientMode(const String& ssid, const String& pass);
    void resetToApMode();
}
