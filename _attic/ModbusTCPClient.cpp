#include "ModbusTCPClient.h"

namespace MetaSense {

ModbusClient::ModbusClient(const char* host, uint16_t port)
    : _host(host),
      _port(port),
      _client(_wifi)   // CORRECT: eModbus requires a Client& here
{
}

bool ModbusClient::connect()
{
    IPAddress ip;
    if (!ip.fromString(_host)) {
        Serial.println("[ModbusClient] Invalid IP address string");
        return false;
    }

    // Set target host + port
    _client.setTarget(ip, _port);

    // Connect if needed
    if (!_client.connected()) {
        if (!_client.connect()) {
            Serial.println("[ModbusClient] Connection failed");
            return false;
        }
    }

    return true;
}

bool ModbusClient::readHolding(uint16_t addr, uint16_t count, uint16_t* buffer)
{
    if (!connect()) return false;

    ModbusMessage response;
    Error err = _client.readHoldingRegisters(addr, count, response);

    if (err != SUCCESS) {
        Serial.printf("[ModbusClient] Read failed: %d\n", err);
        return false;
    }

    for (int i = 0; i < count; i++) {
        buffer[i] = response.getUInt16(i * 2);
    }

    return true;
}

bool ModbusClient::writeHolding(uint16_t addr, uint16_t value)
{
    if (!connect()) return false;

    Error err = _client.writeSingleRegister(addr, value);

    if (err != SUCCESS) {
        Serial.printf("[ModbusClient] Write failed: %d\n", err);
        return false;
    }

    return true;
}

}
