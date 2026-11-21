#pragma once
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NAU7802.h>
#include "HalTypes.h"

namespace MetaSense {

class HalSensors {
public:
    bool begin(TwoWire& wire = Wire);

    bool readEnvironment(EnvSample& out);
    bool readLoadCell(LoadCellSample& out);

    void tareLoadCell();
    void setLoadCellScale(float kgPerCount);

private:
    Adafruit_AHTX0   aht_;
    Adafruit_BMP280  bmp_;
    Adafruit_NAU7802 nau_;

    bool ahtOk_  = false;
    bool bmpOk_  = false;
    bool nauOk_  = false;

    float scaleKgPerCount_ = 1.0f;
    long  offset_          = 0;
};

} // namespace MetaSense
