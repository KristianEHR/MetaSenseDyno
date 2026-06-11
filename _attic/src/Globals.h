#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>

namespace MetaSense {

extern AsyncWebServer server;
extern AsyncWebSocket ws;

extern bool isRecording;

} // namespace MetaSense
