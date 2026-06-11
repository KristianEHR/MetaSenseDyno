#pragma once
#include <Adafruit_MCP9600.h>

class TempHAL {
public:
    bool begin();
    float readC();
private:
    Adafruit_MCP9600 mcp;
};
