#include "TempHAL.h"

#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint8_t kPreferredAddress = MCP9600_I2CADDR_DEFAULT;
constexpr uint8_t kProbeAddresses[] = {
    kPreferredAddress,
    0x60,
    0x61,
    0x62,
    0x63,
    0x64,
    0x65,
    0x66,
};

constexpr uint8_t kExpectedDeviceId = 0x40;
constexpr uint8_t kSensorConfigValue = (MCP9600_TYPE_K << 4) | 0x03;
constexpr uint8_t kDeviceConfigInitValue = 0x80;
constexpr uint8_t kDeviceConfigRunValue = 0x40;
constexpr uint8_t kBootInitPasses = 4;
constexpr uint16_t kBootRetryDelayMs = 120;

bool isFiniteTemp(float value)
{
    return isfinite(value) && value > -100.0f && value < 2000.0f;
}

} // namespace

bool TempHAL::readRegister(uint8_t address, uint8_t reg, uint8_t* data, size_t len) const
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) {
        return false;
    }

    const size_t received = Wire.requestFrom(static_cast<int>(address), static_cast<int>(len), static_cast<int>(true));
    if (received != len) {
        while (Wire.available()) {
            (void)Wire.read();
        }
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        if (!Wire.available()) {
            return false;
        }
        data[i] = static_cast<uint8_t>(Wire.read());
    }

    return true;
}

bool TempHAL::writeRegister8(uint8_t address, uint8_t reg, uint8_t value) const
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

bool TempHAL::readRegister16(uint8_t address, uint8_t reg, int16_t& value) const
{
    uint8_t buffer[2] = {0, 0};
    if (!readRegister(address, reg, buffer, sizeof(buffer))) {
        return false;
    }

    value = static_cast<int16_t>((static_cast<uint16_t>(buffer[0]) << 8) | buffer[1]);
    return true;
}

bool TempHAL::configureSensor(uint8_t address)
{
    uint8_t idBuffer[2] = {0, 0};
    bool idReadOk = false;

    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (readRegister(address, MCP9600_DEVICEID, idBuffer, sizeof(idBuffer))) {
            idReadOk = true;
            break;
        }
        delay(2);
    }

    if (!idReadOk || idBuffer[0] != kExpectedDeviceId) {
        return false;
    }

    if (!writeRegister8(address, MCP9600_DEVICECONFIG, kDeviceConfigInitValue)) {
        return false;
    }
    delay(5);

    if (!writeRegister8(address, MCP9600_SENSORCONFIG, kSensorConfigValue)) {
        return false;
    }

    if (!writeRegister8(address, MCP9600_DEVICECONFIG, kDeviceConfigRunValue)) {
        return false;
    }
    delay(20);

    int16_t hotRaw = 0;
    int16_t ambientRaw = 0;
    if (!readRegister16(address, MCP9600_HOTJUNCTION, hotRaw) ||
        !readRegister16(address, MCP9600_COLDJUNCTION, ambientRaw)) {
        return false;
    }

    const float hotC = static_cast<float>(hotRaw) * 0.0625f;
    const float ambientC = static_cast<float>(ambientRaw) * 0.0625f;
    if (!isFiniteTemp(hotC) || !isFiniteTemp(ambientC)) {
        return false;
    }

    return true;
}

bool TempHAL::begin()
{
    ready = false;
    activeAddress = 0;
    lastAckAddress = 0;
    uint8_t retriesUsed = 0;

    for (uint8_t pass = 0; pass < kBootInitPasses; ++pass) {
        for (uint8_t address : kProbeAddresses) {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() != 0) {
                continue;
            }

            lastAckAddress = address;
            Serial.printf("[TempHAL] MCP9600 ACK at 0x%02X\n", address);
            Serial0.printf("[TempHAL] MCP9600 ACK at 0x%02X\n", address);

            if (configureSensor(address)) {
                ready = true;
                activeAddress = address;
                retriesUsed = pass;
                if (retriesUsed > 0) {
                    Serial.printf("[TempHAL] MCP9600 boot retries used: %u\n", static_cast<unsigned>(retriesUsed));
                    Serial0.printf("[TempHAL] MCP9600 boot retries used: %u\n", static_cast<unsigned>(retriesUsed));
                }
                Serial.printf("[TempHAL] MCP9600 EGT sensor ready at 0x%02X (ID=0x40)\n", address);
                Serial0.printf("[TempHAL] MCP9600 EGT sensor ready at 0x%02X (ID=0x40)\n", address);
                return true;
            }

            Serial.printf("[TempHAL] MCP9600 device at 0x%02X rejected init\n", address);
            Serial0.printf("[TempHAL] MCP9600 device at 0x%02X rejected init\n", address);
        }

        if (pass + 1 < kBootInitPasses) {
            delay(kBootRetryDelayMs);
        }
    }

    Serial.println("[TempHAL] MCP9600 not found on I2C bus (0x67 first, then 0x60-0x66)");
    Serial0.println("[TempHAL] MCP9600 not found on I2C bus (0x67 first, then 0x60-0x66)");
    return false;
}

float TempHAL::readHotC()
{
    if (!ready) {
        return NAN;
    }

    int16_t raw = 0;
    if (!readRegister16(activeAddress, MCP9600_HOTJUNCTION, raw)) {
        ready = false;
        return NAN;
    }

    const float value = static_cast<float>(raw) * 0.0625f;
    if (!isFiniteTemp(value)) {
        ready = false;
        return NAN;
    }

    return value;
}

float TempHAL::readAmbientC()
{
    if (!ready) {
        return NAN;
    }

    int16_t raw = 0;
    if (!readRegister16(activeAddress, MCP9600_COLDJUNCTION, raw)) {
        ready = false;
        return NAN;
    }

    const float value = static_cast<float>(raw) * 0.0625f;
    if (!isFiniteTemp(value)) {
        ready = false;
        return NAN;
    }

    return value;
}

uint8_t TempHAL::status() const
{
    if (!ready) {
        return 0;
    }

    uint8_t value = 0;
    if (!readRegister(activeAddress, MCP9600_STATUS, &value, 1)) {
        return 0;
    }

    return value;
}

int TempHAL::address() const
{
    return ready ? activeAddress : -1;
}

bool TempHAL::isReady() const
{
    return ready;
}

int TempHAL::ackAddress() const
{
    return lastAckAddress ? lastAckAddress : -1;
}
