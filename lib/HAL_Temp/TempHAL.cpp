#include "TempHAL.h"

bool TempHAL::begin() {
    return mcp.begin();
}

float TempHAL::readC() {
    return mcp.readThermocouple();
}
