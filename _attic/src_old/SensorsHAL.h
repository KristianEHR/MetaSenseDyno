#pragma once
#include <stdint.h>

namespace Sensors {

struct Sample {
    float load_N;
    float speed_rpm;
    float temp_C;
    float pressure_hPa;
    float humidity_pct;
    uint32_t timestamp_ms;
};

bool init();
bool read(Sample& out);
bool tareLoadCell();
}
