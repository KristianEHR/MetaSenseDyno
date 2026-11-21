#pragma once

#include <stdint.h>

#include "LeafCan.h"

namespace MetaSense::CANBus {

struct Config {
	int txPin = 4;
	int rxPin = 5;
	uint8_t maxFramesPerPoll = 64;  // Drain 64 frames per poll to minimize RX latency for PI loop
	uint32_t initRetryMs = 2000;
};

// Stats covers only the two frame families this project actually consumes:
// 0x1DA (motor feedback, CRC-validated) and 0x55A (inverter/stator/coolant
// temperatures). All other CAN IDs are ignored on receive.
struct Stats {
	bool ready = false;
	uint32_t lastInitAttemptMs = 0;
	uint32_t lastRxMs = 0;
	uint32_t rxFrames = 0;
	uint32_t rxLeafFrames = 0;
	uint32_t rx1daFrames = 0;
	uint32_t rx55aFrames = 0;
	uint32_t last55aMs = 0;
	uint32_t last1daMs = 0;
	uint8_t last55aLen = 0;
	uint8_t last1daLen = 0;
	int8_t last1daWireCrcOk = -1;
	uint8_t last1daWireCrcCalc = 0;
	uint32_t rx1daCrcOkFrames = 0;      // 0x1DA frames with valid CRC
	uint32_t rx1daCrcBadFrames = 0;     // 0x1DA frames with invalid CRC
	uint8_t last1daData[8] = {0};
	uint8_t last55aData[8] = {0};
	uint32_t recoveries = 0;
	uint32_t busOffEvents = 0;
	uint32_t statusQueryFailures = 0;
	uint8_t lastTwaiState = 0xFF;
	uint32_t twaiRxQueued = 0;
	uint32_t twaiTxQueued = 0;
	uint32_t twaiRxMissed = 0;
	uint32_t twaiRxOverrun = 0;
	uint32_t twaiArbLost = 0;
	uint32_t twaiBusError = 0;
	uint32_t twaiTxErrorCounter = 0;
	uint32_t twaiRxErrorCounter = 0;
	uint32_t rx1daWireCrcOkFrames = 0;
	uint32_t rx1daWireCrcBadFrames = 0;
};

void configure(const Config& config);
void reset();
void poll(uint32_t nowMs);
bool send(uint32_t id, const uint8_t* data, uint8_t len);
bool isReady();
const LeafInvFeedback& feedback();
const Stats& stats();

} // namespace MetaSense::CANBus
