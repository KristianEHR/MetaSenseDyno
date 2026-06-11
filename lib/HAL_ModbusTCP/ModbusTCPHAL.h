#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoModbus.h>

class ModbusTCPHAL {
public:
    bool beginServer(uint16_t port = 502);
    bool beginClient(IPAddress host, uint16_t port = 502);

    // Server-side register access
    void setHoldingRegister(uint16_t reg, uint16_t value);
    uint16_t getHoldingRegister(uint16_t reg);

    // Client-side operations
    bool clientWrite(uint16_t reg, uint16_t value);
    bool clientRead(uint16_t reg, uint16_t count, uint16_t* dest);

    void poll(); // must be called frequently

private:
    bool serverMode = false;
    bool clientMode = false;
};
