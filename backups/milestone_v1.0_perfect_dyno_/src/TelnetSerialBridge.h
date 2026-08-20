#pragma once

#include <Arduino.h>
#include <AsyncTCP.h>
#include <vector>
#include <deque>
#include <cstdarg>

namespace MetaSense::TelnetSerialBridge {

class TelnetSerialBridgeService {
public:
    TelnetSerialBridgeService() = default;
    ~TelnetSerialBridgeService() = default;
    
    // Initialize telnet server on specified port
    bool begin(uint16_t port = 23);
    
    // Stop telnet server
    void end();
    
    // Write data to all connected telnet clients
    void writeToClients(const uint8_t* data, size_t len);
    
    // Get number of connected clients
    size_t getClientCount() const { return clients.size(); }
    
    // Update method to be called periodically
    void update();
    
private:
    AsyncServer* server = nullptr;
    uint16_t listenPort = 23;
    std::vector<AsyncClient*> clients;
    std::deque<uint8_t> buffer;
    static const size_t MAX_BUFFER_SIZE = 4096;
    
    void onNewClient(void* arg, AsyncClient* client);
    void onClientData(AsyncClient* client, void* data, size_t len);
    void onClientDisconnect(AsyncClient* client);
    void onClientError(AsyncClient* client, int8_t error);
    void onClientTimeout(AsyncClient* client, uint32_t time);
    
    static void staticOnNewClient(void* arg, AsyncClient* client);
    static void staticOnClientData(void* arg, AsyncClient* client, void* data, size_t len);
    static void staticOnClientDisconnect(void* arg, AsyncClient* client);
    static void staticOnClientError(void* arg, AsyncClient* client, int8_t error);
    static void staticOnClientTimeout(void* arg, AsyncClient* client, uint32_t time);
};

// Global instance
extern TelnetSerialBridgeService telnetSerialBridge;

// Inline helper for formatted telnet output
inline void telnetBridgePrintf(const char* format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    telnetSerialBridge.writeToClients((const uint8_t*)buffer, strlen(buffer));
}

} // namespace MetaSense::TelnetSerialBridge
