#pragma once

class AsyncWebSocketClient;
class String;

namespace MetaSense::CommandRouter {

void handleWebSocketMessage(AsyncWebSocketClient* client, const String& msg);
bool loadFactoryProfileOnBoot();

} // namespace MetaSense::CommandRouter