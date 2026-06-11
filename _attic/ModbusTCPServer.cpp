#include "ModbusTCPServer.h"

namespace MetaSense {

ModbusServer::ModbusServer(uint16_t port)
    : _port(port)
{
    // Constructor must be empty because async server takes no arguments
}

bool ModbusServer::begin(uint16_t holdingCount)
{
    // Set the port AFTER construction
    _server.setPort(_port);

    if (!_server.start()) {
        Serial.println("[ModbusServer] Failed to start async server");
        return false;
    }

    // Create holding registers
    _server.addHoldingRegisters(0, holdingCount);

    Serial.println("[ModbusServer] Async server started");
    return true;
}

void ModbusServer::update()
{
    // Async server does not require polling
}

void ModbusServer::writeHolding(uint16_t addr, uint16_t value)
{
    _server.writeHoldingRegister(addr, value);
}

}
