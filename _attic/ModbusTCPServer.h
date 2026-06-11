#pragma once
#include <ModbusServerTCPasync.h>

namespace MetaSense {

class ModbusServer {
public:
    ModbusServer(uint16_t port = 502);

    bool begin(uint16_t holdingCount);
    void update();
    void writeHolding(uint16_t addr, uint16_t value);

private:
    uint16_t _port;
    ModbusServerTCPasync _server;
};

}
