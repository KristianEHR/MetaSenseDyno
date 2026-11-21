#include "TelnetSerialBridge.h"
#include <algorithm>

namespace MetaSense::TelnetSerialBridge {

// Global instance
TelnetSerialBridgeService telnetSerialBridge;

bool TelnetSerialBridgeService::begin(uint16_t port) {
    listenPort = port;
    
    try {
        server = new AsyncServer(listenPort);
        
        server->onClient(
            [this](void* arg, AsyncClient* client) {
                this->onNewClient(arg, client);
            },
            this
        );
        
        server->begin();
        
        Serial.printf("[TelnetBridge] Started on port %u\n", listenPort);
        return true;
    } catch (...) {
        Serial.printf("[TelnetBridge] Failed to start on port %u\n", listenPort);
        return false;
    }
}

void TelnetSerialBridgeService::end() {
    if (server) {
        server->end();
        delete server;
        server = nullptr;
    }
    
    // Disconnect all clients
    for (auto* client : clients) {
        if (client && client->connected()) {
            client->close();
        }
    }
    clients.clear();
}

void TelnetSerialBridgeService::writeToClients(const uint8_t* data, size_t len) {
    // Add to buffer
    for (size_t i = 0; i < len; ++i) {
        if (buffer.size() < MAX_BUFFER_SIZE) {
            buffer.push_back(data[i]);
        } else {
            buffer.pop_front();
            buffer.push_back(data[i]);
        }
    }
    
    // Send to all connected clients
    for (auto* client : clients) {
        if (client && client->connected()) {
            client->write((const char*)data, len);
        }
    }
}

void TelnetSerialBridgeService::update() {
    // Clean up disconnected clients
    clients.erase(
        std::remove_if(clients.begin(), clients.end(),
            [](AsyncClient* client) {
                return !client || !client->connected();
            }
        ),
        clients.end()
    );
}

void TelnetSerialBridgeService::onNewClient(void* arg, AsyncClient* client) {
    if (!client) return;
    
    Serial.printf("[TelnetBridge] New client connected: %s\n", client->remoteIP().toString().c_str());
    
    clients.push_back(client);
    
    // Send welcome message
    const char* welcome = "\r\nMetaSense Telnet Serial Bridge\r\n";
    client->write(welcome, strlen(welcome));
    
    // Send buffered data to new client
    if (!buffer.empty()) {
        std::vector<uint8_t> bufferCopy(buffer.begin(), buffer.end());
        client->write((const char*)bufferCopy.data(), bufferCopy.size());
    }
    
    // Set up callbacks
    client->onData(
        [this](void* arg, AsyncClient* client, void* data, size_t len) {
            this->onClientData(client, data, len);
        },
        this
    );
    
    client->onDisconnect(
        [this](void* arg, AsyncClient* client) {
            this->onClientDisconnect(client);
        },
        this
    );
    
    client->onError(
        [this](void* arg, AsyncClient* client, int8_t error) {
            this->onClientError(client, error);
        },
        this
    );
    
    client->onTimeout(
        [this](void* arg, AsyncClient* client, uint32_t time) {
            this->onClientTimeout(client, time);
        },
        this
    );
}

void TelnetSerialBridgeService::onClientData(AsyncClient* client, void* data, size_t len) {
    // Echo data back (optional telnet interaction)
    // For now, just ignore client input
}

void TelnetSerialBridgeService::onClientDisconnect(AsyncClient* client) {
    if (!client) return;
    
    Serial.printf("[TelnetBridge] Client disconnected: %s\n", client->remoteIP().toString().c_str());
    
    auto it = std::find(clients.begin(), clients.end(), client);
    if (it != clients.end()) {
        clients.erase(it);
    }
}

void TelnetSerialBridgeService::onClientError(AsyncClient* client, int8_t error) {
    if (!client) return;
    
    Serial.printf("[TelnetBridge] Client error %d: %s\n", error, client->remoteIP().toString().c_str());
    
    auto it = std::find(clients.begin(), clients.end(), client);
    if (it != clients.end()) {
        clients.erase(it);
    }
}

void TelnetSerialBridgeService::onClientTimeout(AsyncClient* client, uint32_t time) {
    if (!client) return;
    
    Serial.printf("[TelnetBridge] Client timeout: %s (%u ms)\n", client->remoteIP().toString().c_str(), time);
    
    if (client->connected()) {
        client->close();
    }
    
    auto it = std::find(clients.begin(), clients.end(), client);
    if (it != clients.end()) {
        clients.erase(it);
    }
}

} // namespace MetaSense::TelnetSerialBridge
