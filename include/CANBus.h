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
	uint32_t rxStdFrames = 0;
	uint32_t rxExtFrames = 0;
	uint32_t rxLeafFrames = 0;
	uint32_t rx1daFrames = 0;
	uint32_t rx1dcFrames = 0;
	uint32_t rxUnknownFrames = 0;
	uint32_t rx120Frames = 0;
	uint32_t rx1dbFrames = 0;
	uint32_t rx11aFrames = 0;
	uint32_t rx120Changes = 0;
	uint32_t rx11aChanges = 0;
	uint32_t rx55aFrames = 0;
	uint32_t rx55bFrames = 0;
	uint32_t rx05bFrames = 0;
	uint32_t rx50bFrames = 0;
	uint32_t lastUnknownMs = 0;
	uint32_t lastUnknownId = 0;
	uint32_t last55bSrcId = 0;
	uint32_t last55aMs = 0;
	uint32_t last55bMs = 0;
	uint32_t last11aMs = 0;
	uint32_t last120Ms = 0;
	uint8_t lastUnknownLen = 0;
	uint8_t last55aLen = 0;
	uint8_t last55bLen = 0;
	uint8_t last11aLen = 0;
	uint8_t last120Len = 0;
	uint8_t last11aChangeMask = 0;
	uint8_t last120ChangeMask = 0;
	uint8_t agg11aChangeMask = 0;
	uint8_t agg120ChangeMask = 0;
	uint8_t lastUnknownData[8] = {0};
	uint8_t last55aData[8] = {0};
	uint8_t last55bData[8] = {0};
	uint8_t last11aData[8] = {0};
	uint8_t last120Data[8] = {0};
	uint32_t byteChg11a[8] = {0};
	uint32_t byteChg120[8] = {0};
	uint32_t lastRxId = 0;
	uint32_t txFrames = 0;
	uint32_t txFailures = 0;
	uint32_t txWhileNotReady = 0;
	bool txFailureLatched = false;
	bool txWhileNotReadyLatched = false;
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
};

void configure(const Config& config);
void reset();
void poll(uint32_t nowMs);
bool send(uint32_t id, const uint8_t* data, uint8_t len);
bool isReady();
const LeafInvFeedback& feedback();
const Stats& stats();

} // namespace MetaSense::CANBus
