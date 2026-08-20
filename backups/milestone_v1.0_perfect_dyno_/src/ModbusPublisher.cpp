#include "ModbusPublisher.h"
#include "Input.h"
#include "RunStorage.h"
#include "Settings.h"

#include <math.h>

namespace {

constexpr uint16_t kLegacyFwMajor = 3;
constexpr uint16_t kLegacyFwMinor = 0;
constexpr uint16_t kStableMapVersion = 5;
constexpr uint16_t kLegacyPublishPeriodMs = 50;
constexpr uint16_t kLegacyRegisterCount = 30;

uint16_t toRegisterValueSigned(float value, float scale = 1.0f)
{
    const float scaled = value * scale;
    long rounded = lroundf(scaled);
    if (rounded < -32768L) {
        rounded = -32768L;
    } else if (rounded > 32767L) {
        rounded = 32767L;
    }
    return static_cast<uint16_t>(static_cast<int16_t>(rounded));
}

uint16_t telemetryStatusWord(const MetaSense::Telemetry& telemetry)
{
    uint16_t status = 0;
    if (telemetry.recording) {
        status |= 0x0001U;
    }
    status |= (static_cast<uint16_t>(telemetry.mode) & 0x0003U) << 1;
    return status;
}

float egtCompatC(const MetaSense::Telemetry& telemetry)
{
    if (isfinite(telemetry.egtHotC) && telemetry.egtHotC > -50.0f && telemetry.egtHotC < 1800.0f) {
        return telemetry.egtHotC;
    }
    if (isfinite(telemetry.egtAmbientC) && telemetry.egtAmbientC > -50.0f && telemetry.egtAmbientC < 200.0f) {
        return telemetry.egtAmbientC;
    }
    return 0.0f;
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
    const uint32_t currentVersion = MetaSense::RunStorage::version();
    const MetaSense::Telemetry telemetry = MetaSense::RunStorage::latest();

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
    if (now - _lastUpdate < 50 && currentVersion == _lastUpdateVersion) return;
    _lastUpdate = now;
    _lastUpdateVersion = currentVersion;

    if (_regCount < 30) {
        return;
    }

    // Legacy/meta block: registers 0-9
    server.holdingRegisterWrite(0, kLegacyFwMajor);
    server.holdingRegisterWrite(1, kLegacyFwMinor);
    server.holdingRegisterWrite(2, kStableMapVersion);
    server.holdingRegisterWrite(3, kLegacyPublishPeriodMs);
    server.holdingRegisterWrite(4, toRegisterValueSigned(telemetry.energyMJ, 100.0f));
    // Compatibility mirror for legacy clients polling low addresses.
    server.holdingRegisterWrite(5, toRegisterValueSigned(telemetry.throttlePercent, 10.0f));
    server.holdingRegisterWrite(6, toRegisterValueSigned(telemetry.rpmTarget));
    server.holdingRegisterWrite(7, toRegisterValueSigned(MetaSense::Input::currentKpLive(), 10000.0f));
    server.holdingRegisterWrite(8, toRegisterValueSigned(MetaSense::Settings::ki, 10000.0f));
    server.holdingRegisterWrite(9, toRegisterValueSigned(egtCompatC(telemetry), 10.0f));

    // Live dyno data: registers 10-29
    server.holdingRegisterWrite(10, toRegisterValueSigned(telemetry.rpm));
    server.holdingRegisterWrite(11, toRegisterValueSigned(telemetry.drumRpm));
    server.holdingRegisterWrite(12, toRegisterValueSigned(telemetry.loadKg, 10.0f));
    server.holdingRegisterWrite(13, toRegisterValueSigned(telemetry.torqueNm, 10.0f));
    server.holdingRegisterWrite(14, toRegisterValueSigned(telemetry.brakeTorqueNm, 10.0f));
    server.holdingRegisterWrite(15, toRegisterValueSigned(telemetry.energyMJ, 100.0f));
    server.holdingRegisterWrite(16, toRegisterValueSigned(telemetry.airDensity, 1000.0f));
    server.holdingRegisterWrite(17, toRegisterValueSigned(telemetry.ambientC, 10.0f));
    server.holdingRegisterWrite(18, toRegisterValueSigned(telemetry.pressureHpa, 10.0f));
    server.holdingRegisterWrite(19, toRegisterValueSigned(telemetry.egtHotC, 10.0f));
    server.holdingRegisterWrite(20, toRegisterValueSigned(telemetry.egtAmbientC, 10.0f));
    server.holdingRegisterWrite(21, toRegisterValueSigned(telemetry.peakTorque, 10.0f));
    server.holdingRegisterWrite(22, toRegisterValueSigned(telemetry.peakTorque_RPM));
    server.holdingRegisterWrite(23, toRegisterValueSigned(telemetry.maxRpm));
    server.holdingRegisterWrite(24, toRegisterValueSigned(telemetry.maxTorqueNm, 10.0f));
    server.holdingRegisterWrite(25, toRegisterValueSigned(telemetry.rpmTarget));
    server.holdingRegisterWrite(26, toRegisterValueSigned(telemetry.kw, 100.0f));
    server.holdingRegisterWrite(27, toRegisterValueSigned(telemetry.humidity, 10.0f));
    server.holdingRegisterWrite(28, toRegisterValueSigned(telemetry.eTorque, 10.0f));
    server.holdingRegisterWrite(29, telemetryStatusWord(telemetry));
}

} // namespace MetaSense
