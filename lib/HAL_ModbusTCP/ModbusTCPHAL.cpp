#include "ModbusTCPHAL.h"

bool ModbusTCPHAL::beginServer(uint16_t port) {
    if (!ModbusTCPServer.begin(port)) return false;
    ModbusTCPServer.configureHoldingRegisters(0, 200);
    serverMode = true;
    return true;
}

bool ModbusTCPHAL::beginClient(IPAddress host, uint16_t port) {
    if (!ModbusTCPClient.begin(host, port)) return false;
    clientMode = true;
    return true;
}

void ModbusTCPHAL::setHoldingRegister(uint16_t reg, uint16_t value) {
    if (serverMode) ModbusTCPServer.holdingRegisterWrite(reg, value);
}

uint16_t ModbusTCPHAL::getHoldingRegister(uint16_t reg) {
    if (serverMode) return ModbusTCPServer.holdingRegisterRead(reg);
    return 0;
}

bool ModbusTCPHAL::clientWrite(uint16_t reg, uint16_t value) {
    if (!clientMode) return false;
    return ModbusTCPClient.holdingRegisterWrite(reg, value);
}

bool ModbusTCPHAL::clientRead(uint16_t reg, uint16_t count, uint16_t* dest) {
    if (!clientMode) return false;
    if (!ModbusTCPClient.requestFrom(reg, count)) return false;

    for (uint16_t i = 0; i < count; i++)
        dest[i] = ModbusTCPClient.read();

    return true;
}

void ModbusTCPHAL::poll() {
    if (serverMode) ModbusTCPServer.poll();
}
