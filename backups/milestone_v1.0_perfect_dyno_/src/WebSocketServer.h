#pragma once

class AsyncWebServer;
class AsyncWebSocket;

namespace MetaSense::WebSocketServer {

AsyncWebSocket& socket();
void begin(AsyncWebServer& server);
void loop();

} // namespace MetaSense::WebSocketServer