#pragma once
#include <driver/twai.h>

class CanHAL {
public:
    bool begin(int txPin, int rxPin);
    void stop();
    bool send(uint32_t id, const uint8_t* data, uint8_t len);
    bool receive(uint32_t& id, uint8_t* data, uint8_t& len, bool& isExtended);
    bool getStatus(twai_status_info_t& outStatus) const;

private:
    bool started_ = false;
    int txPin_ = -1;
    int rxPin_ = -1;
};
