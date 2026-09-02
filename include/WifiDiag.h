#pragma once

#include <cstdint>

// Tracks the most recent WiFi STA disconnect reason so it can be surfaced to
// the GUI (as a toast) after the WebSocket reconnects -- the browser has no
// way to see this itself since the WebSocket link is down for the duration
// of the actual disconnect/reassociation.
namespace MetaSense::WifiDiag {

// Records a disconnect event (reason code from WiFiEventInfo_t::wifi_sta_disconnected.reason)
// and bumps the event sequence number so consumers can detect a *new* event
// even if the reason code repeats.
void recordDisconnect(int16_t reasonCode);

// -1 if no disconnect has been recorded yet this boot.
int16_t lastDisconnectReason();

// Monotonically increasing counter, incremented once per recorded disconnect.
uint32_t disconnectEventSeq();

} // namespace MetaSense::WifiDiag
