#include "WifiDiag.h"

namespace MetaSense::WifiDiag {

namespace {
int16_t s_lastDisconnectReason = -1;
uint32_t s_disconnectEventSeq = 0;
} // namespace

void recordDisconnect(int16_t reasonCode)
{
    s_lastDisconnectReason = reasonCode;
    ++s_disconnectEventSeq;
}

int16_t lastDisconnectReason()
{
    return s_lastDisconnectReason;
}

uint32_t disconnectEventSeq()
{
    return s_disconnectEventSeq;
}

} // namespace MetaSense::WifiDiag
