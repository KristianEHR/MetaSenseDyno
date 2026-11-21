#include "Globals.h"

namespace MetaSense {

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool isRecording = false;


} // namespace MetaSense
