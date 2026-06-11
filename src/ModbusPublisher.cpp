#include "ModbusPublisher.h"
#include "RunStorage.h"

#include <math.h>

namespace {

uint16_t toRegisterValue(float value)
{
    long rounded = lroundf(value);
    if (rounded < 0) {
        return 0;
    }
    if (rounded > 65535L) {
        return 65535;
    }
    return static_cast<uint16_t>(rounded);
}

uint16_t toRegisterValue(bool value)
{
    return value ? 1U : 0U;
}

} // namespace

namespace MetaSense {

bool ModbusPublisher::begin(uint16_t port, uint16_t regCount)
{
    _regCount = regCount;

    if (_wifiServer != nullptr) {
        _wifiServer->stop();
        delete _wifiServer;
        _wifiServer = nullptr;
    }

    _wifiServer = new WiFiServer(port);
    _wifiServer->begin();

    if (!server.begin()) {
        Serial.println("[ModbusPublisher] Failed to start Modbus TCP Server");
        return false;
    }

    server.configureHoldingRegisters(0, _regCount);

    Serial.printf("[ModbusPublisher] Started on port %u with %u registers\n",
                  port, _regCount);

    return true;
}

void ModbusPublisher::update()
{
    const MetaSense::Telemetry& telemetry = MetaSense::RunStorage::latest();

    if (_wifiServer != nullptr) {
        WiFiClient nextClient = _wifiServer->available();
        if (nextClient) {
            _client = nextClient;
            server.accept(_client);
        }

        if (_client.connected()) {
            server.poll();
        }
    }

    unsigned long now = millis();
    if (now - _lastUpdate < 50) return;   // 50 ms update rate
    _lastUpdate = now;

    if (_regCount < 21) {
        return;
    }

    server.holdingRegisterWrite(0, toRegisterValue(telemetry.rpm));
    server.holdingRegisterWrite(1, toRegisterValue(telemetry.drumRpm));
    server.holdingRegisterWrite(2, toRegisterValue(telemetry.loadKg));
    server.holdingRegisterWrite(3, toRegisterValue(telemetry.torqueNm));
    server.holdingRegisterWrite(4, toRegisterValue(telemetry.brakeTorqueNm));
    server.holdingRegisterWrite(5, toRegisterValue(telemetry.energyMJ));
    server.holdingRegisterWrite(6, toRegisterValue(telemetry.airDensity));
    server.holdingRegisterWrite(7, toRegisterValue(telemetry.ambientC));
    server.holdingRegisterWrite(8, toRegisterValue(telemetry.pressureHpa));
    server.holdingRegisterWrite(9, toRegisterValue(telemetry.egtHotC));
    server.holdingRegisterWrite(10, toRegisterValue(telemetry.egtAmbientC));
    server.holdingRegisterWrite(11, toRegisterValue(telemetry.recording));
    server.holdingRegisterWrite(12, toRegisterValue(telemetry.peakTorque));
    server.holdingRegisterWrite(13, toRegisterValue(telemetry.peakTorque_RPM));
    server.holdingRegisterWrite(14, toRegisterValue(telemetry.maxRpm));
    server.holdingRegisterWrite(15, toRegisterValue(telemetry.maxTorqueNm));
    server.holdingRegisterWrite(16, toRegisterValue(telemetry.rpmTarget));
    server.holdingRegisterWrite(17, toRegisterValue(telemetry.kw));
    server.holdingRegisterWrite(18, toRegisterValue(telemetry.humidity));
    server.holdingRegisterWrite(19, toRegisterValue(telemetry.eTorque));
    server.holdingRegisterWrite(20, static_cast<uint16_t>(telemetry.mode));
}

} // namespace MetaSense
