#pragma once
#include <ModbusClientTCP.h>
#include <WiFi.h>

namespace MetaSense {

class ModbusClient {
public:
    ModbusClient(const char* host, uint16_t port);

    bool connect();
    bool readHolding(uint16_t addr, uint16_t count, uint16_t* buffer);
    bool writeHolding(uint16_t addr, uint16_t value);

private:
    const char* _host;
    uint16_t _port;

    WiFiClient _wifi;
    ModbusClientTCP _client;
};

}
