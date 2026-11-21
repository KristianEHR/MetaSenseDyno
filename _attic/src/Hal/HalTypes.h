#pragma once

namespace MetaSense {

struct EnvSample {
    float temperatureC;
    float humidityPct;
    float pressurePa;
};

struct LoadCellSample {
    float raw;
    float kg;
};

} // namespace MetaSense
