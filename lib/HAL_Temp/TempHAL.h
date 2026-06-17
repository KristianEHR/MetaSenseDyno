#pragma once

#include <stdint.h>

#include <Adafruit_MCP9600.h>

class TempHAL {
public:
    bool begin();
    float readHotC();
    float readAmbientC();
    uint8_t status() const;
    int address() const;
    bool isReady() const;
    int ackAddress() const;

private:
    bool configureSensor(uint8_t address);
    bool readRegister(uint8_t address, uint8_t reg, uint8_t* data, size_t len) const;
    bool writeRegister8(uint8_t address, uint8_t reg, uint8_t value) const;
    bool readRegister16(uint8_t address, uint8_t reg, int16_t& value) const;

    bool ready = false;
    uint8_t activeAddress = 0;
    uint8_t lastAckAddress = 0;
};
