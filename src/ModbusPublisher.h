#pragma once
#include <ModbusTCPServer.h>
#include <WiFi.h>

namespace MetaSense {

class ModbusPublisher {
public:
    bool begin(uint16_t port = 503, uint16_t regCount = 21);
    void update();

private:
    ModbusTCPServer server;
    WiFiServer* _wifiServer = nullptr;
    WiFiClient _client;
    uint16_t _regCount = 0;
    unsigned long _lastUpdate = 0;
};

} // namespace MetaSense
