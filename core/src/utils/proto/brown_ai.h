#pragma once

#include <dsp/types.h>
#include <complex>
#include <atomic>
#include <ctm.h>
#include <thread>
#include <core.h>
#include <atomic>
#include "picohash.h"
#include "utils/usleep.h"
#include "utils/proto/websock.h"

struct BrownAIClient {
    net::websock::WSClient wsClient;
    std::string hostPort;
    bool connected;
    char connectionStatus[100];
    int64_t lastPing;
    std::atomic<bool> running;
    std::atomic<bool> needsReconnect;
    std::thread connectionThread;
    std::vector<uint8_t> pendingAudioData;
    std::mutex pendingMutex;

    std::function<void()> onConnected = []() {};
    std::function<void()> onDisconnected = []() {};
    std::function<void(const std::string&)> onCommandReceived = [](auto){};

    virtual ~BrownAIClient() {
        stop();
    }

    void init(const std::string& hostport) {
        this->hostPort = hostport;
        strcpy(connectionStatus, "Not connected");

        wsClient.onDisconnected = [&]() {
            connected = false;
            this->onDisconnected();
            snprintf(connectionStatus, sizeof connectionStatus, "Disconnected");
            needsReconnect = true;
        };

        wsClient.onConnected = [&]() {
            connected = true;
            needsReconnect = false;
            
            // Send any pending audio data now that we're connected
            std::lock_guard<std::mutex> lck(pendingMutex);
            if (!pendingAudioData.empty()) {
                wsClient.sendBinary(pendingAudioData);
                pendingAudioData.clear();
            }
            
            if (this->onConnected) {
                onConnected();
            }
            strcpy(connectionStatus, "Connected");
        };

        wsClient.onTextMessage = [&](const std::string& msg) {
            // Handle text responses (commands)
            onCommandReceived(msg);
        };

        wsClient.onBinaryMessage = [&](const std::string& msg) {
            // Handle binary responses if needed
        };

        lastPing = currentTimeMillis();
        wsClient.onEveryReceive = [&]() {
            // Keep connection alive
            // if (currentTimeMillis() - lastPing > 4000) {
            //     wsClient.sendString("PING");
            //     lastPing = currentTimeMillis();
            // }
        };
    }

    void sendAudioData(const std::vector<uint8_t>& audioData) {
        std::lock_guard<std::mutex> lck(pendingMutex);
        
        if (connected) {
            // If we have pending data, send that first
            if (!pendingAudioData.empty()) {
                wsClient.sendBinary(pendingAudioData);
                pendingAudioData.clear();
            }
            
            // Create message with secret key and audio data
            std::string secret = generateSecretKey();
            std::string separator = "----------sdr++--payload---separator-----";
            
            std::vector<uint8_t> message;
            message.insert(message.end(), secret.begin(), secret.end());
            message.insert(message.end(), separator.begin(), separator.end());
            message.insert(message.end(), audioData.begin(), audioData.end());
            
            wsClient.sendBinary(message);
        } else {
            // Buffer the data until we're connected
            std::string secret = generateSecretKey();
            std::string separator = "----------sdr++--payload---separator-----";
            
            pendingAudioData.insert(pendingAudioData.end(), secret.begin(), secret.end());
            pendingAudioData.insert(pendingAudioData.end(), separator.begin(), separator.end());
            pendingAudioData.insert(pendingAudioData.end(), audioData.begin(), audioData.end());
        }
    }

    void stop() {
        strcpy(connectionStatus, "Disconnecting..");
        running = false;
        needsReconnect = false;
        connected = false;
        
        // Clean up WebSocket connection
        wsClient.stopSocket();
        
        // Wait for thread to finish
        if (connectionThread.joinable()) {
            connectionThread.join();
        }
        
        strcpy(connectionStatus, "Disconnected.");
    }

    void start() {
        strcpy(connectionStatus, "Connecting..");
        running = true;
        needsReconnect = true;
        
        connectionThread = std::thread([&]() {
            SetThreadName("brown_ai.wscli");
            while (running) {
                if (needsReconnect) {
                    try {
                        std::string hostName;
                        int port;
                        std::size_t colonPosition = hostPort.find(":");
                        if (colonPosition != std::string::npos) {
                            hostName = hostPort.substr(0, colonPosition);
                            port = std::stoi(hostPort.substr(colonPosition + 1));
                        } else {
                            hostName = hostPort;
                            port = 8080; // Default port
                        }
                
                        // Clear any existing connection
                        wsClient.stopSocket();
                
                        // Attempt connection
                        wsClient.connectAndReceiveLoop(hostName, port, "/ws");
                        strcpy(connectionStatus, "Connected");
                    } catch (const std::runtime_error& e) {
                        std::string errMsg = e.what();
                        flog::error("BrownAIClient: Connection error: {}", errMsg);
                
                        // Handle specific errors
                        if (errMsg.find("recv failed") != std::string::npos) {
                            strcpy(connectionStatus, "Connection lost, reconnecting...");
                        } else {
                            strcpy(connectionStatus, "Error: ");
                            strcat(connectionStatus, errMsg.c_str());
                        }
                
                        // Clean up and wait before retrying
                        wsClient.stopSocket();
                        connected = false;
                        usleep(5000000); // 5 seconds
                    }
                } else {
                    usleep(100000); // 100ms
                }
            }
        });
    }

private:
    std::string generateSecretKey() {
        int64_t now = currentTimeMillis() / 1000; // Current time in seconds
        int64_t window = now / 3600;
        std::string keyBase = std::to_string(window) + "SDR++Brown FTW";
        
        picohash_ctx_t ctx;
        picohash_init_md5(&ctx);
        picohash_update(&ctx, keyBase.c_str(), keyBase.length());
        
        unsigned char hash[PICOHASH_MD5_DIGEST_LENGTH];
        picohash_final(&ctx, hash);
        
        char mdString[33];
        for(int i = 0; i < 16; i++)
            sprintf(&mdString[i*2], "%02x", (unsigned int)hash[i]);
            
        return std::string(mdString);
    }
};
