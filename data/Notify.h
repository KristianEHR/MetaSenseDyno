#pragma once
#include <Arduino.h>
#include "Telemetry.h"

void notifyClients(const Telemetry &data, bool isRecording);
