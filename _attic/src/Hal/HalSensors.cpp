#include "HalSensors.h"
#include <Arduino.h>
#include <Wire.h>

namespace MetaSense {

bool HalSensors::begin(TwoWire &wire)
{
    wire.begin();

    // --- AHT20 (temp + humidity) ---
    ahtOk_ = aht_.begin(&wire);

    // --- BMP280 (pressure + temp) ---
    // Correct Adafruit_BMP280 API: begin(address)
    bmpOk_ = bmp_.begin(0x76);

    // --- NAU7802 (load cell ADC) ---
    // Correct Adafruit_NAU7802 API: begin(TwoWire*)
    nauOk_ = nau_.begin(&wire);

    return ahtOk_ && bmpOk_ && nauOk_;
}

bool HalSensors::readEnvironment(EnvSample &out)
{
    if (!ahtOk_ && !bmpOk_)
        return false;

    // Initialize with NAN so caller can detect missing channels
    out.temperatureC = NAN;
    out.humidityPct  = NAN;
    out.pressurePa   = NAN;

    // --- AHT20 ---
    if (ahtOk_) {
        sensors_event_t hum, temp;
        aht_.getEvent(&hum, &temp);
        out.temperatureC = temp.temperature;
        out.humidityPct  = hum.relative_humidity;
    }

    // --- BMP280 ---
    if (bmpOk_) {
        float p = bmp_.readPressure();
        if (!isnan(p)) {
            out.pressurePa = p;
        }

        // If AHT20 failed, use BMP280 temp as fallback
        if (isnan(out.temperatureC)) {
            float t = bmp_.readTemperature();
            if (!isnan(t)) {
                out.temperatureC = t;
            }
        }
    }

    return true;
}

bool HalSensors::readLoadCell(LoadCellSample &out)
{
    if (!nauOk_)
        return false;

    // NAU7802 returns raw ADC counts
    out.raw = static_cast<float>(nau_.read());

    // Simple placeholder scaling: user will calibrate later
    out.kg = out.raw * scaleKgPerCount_ + offsetKg_;

    return true;
}

void HalSensors::tareLoadCell()
{
    if (!nauOk_)
        return;

    // Simple tare: read once and store as offset
    float raw = static_cast<float>(nau_.read());
    offsetKg_ = -raw * scaleKgPerCount_;
}

} // namespace MetaSense
