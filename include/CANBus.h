#pragma once

#include <stdint.h>

#include "LeafCan.h"

namespace MetaSense::CANBus {

struct Config {
	int txPin = 4;
	int rxPin = 5;
	uint8_t maxFramesPerPoll = 8;
	uint32_t initRetryMs = 2000;
};

struct Stats {
	bool ready = false;
	uint32_t lastInitAttemptMs = 0;
	uint32_t lastRxMs = 0;
	uint32_t rxFrames = 0;
	uint32_t txFrames = 0;
	uint32_t txFailures = 0;
	uint32_t txWhileNotReady = 0;
	bool txFailureLatched = false;
	bool txWhileNotReadyLatched = false;
	uint32_t recoveries = 0;
	uint32_t busOffEvents = 0;
	uint32_t statusQueryFailures = 0;
	uint8_t lastTwaiState = 0xFF;
};

void configure(const Config& config);
void reset();
void poll(uint32_t nowMs);
bool send(uint32_t id, const uint8_t* data, uint8_t len);
bool isReady();
const LeafInvFeedback& feedback();
const Stats& stats();

} // namespace MetaSense::CANBus
