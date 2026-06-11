#pragma once

class AsyncWebSocketClient;
class String;

namespace MetaSense::CommandRouter {

void handleWebSocketMessage(AsyncWebSocketClient* client, const String& msg);

} // namespace MetaSense::CommandRouter