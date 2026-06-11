#include "HalWifi.h"
#include <WiFi.h>

namespace HalWifi {

void setClientMode(const String& ssid, const String& pass) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
}

void resetToApMode() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("MetaSenseDyno", "metasense");
}

}
