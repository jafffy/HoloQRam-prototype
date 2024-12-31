#pragma once

#include "hologram/network/NetworkProtocol.hpp"
#include "hologram/network/NetworkMetrics.hpp"
#include "hologram/network/AdaptiveNetworkControl.hpp"
#include <unordered_map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <queue>
#include <functional>

namespace hologram {

class ReliableNetworkManager {
public:
    ReliableNetworkManager(int sockfd);
    ~ReliableNetworkManager();

    // Start/stop the manager
    void start();
    void stop();

    // Send data reliably
    bool sendReliable(const std::vector<char>& data, PacketType type);
    
    // Set callback for received data
    using DataCallback = std::function<void(const std::vector<char>&, PacketType)>;
    void setDataCallback(DataCallback callback);

    // Get metrics
    PacketMetrics getPacketMetrics() const;
    CompressionMetrics getCompressionMetrics() const;
    AdaptiveNetworkControl::NetworkParams getNetworkParams() const;

    // Network statistics
    float getLastRTT() const { return metrics.getPacketMetrics().averageRoundTripTime; }
    float getPacketLossRate() const { return metrics.getPacketMetrics().packetLossRate; }

private:
    // Network socket
    int sockfd;
    bool running;

    // Sequence numbers
    uint32_t currentSequence;
    uint32_t expectedSequence;
    std::mutex sequenceMutex;

    // Packet tracking
    std::unordered_map<uint32_t, PacketInfo> sentPackets;
    std::unordered_map<uint32_t, std::vector<std::vector<char>>> receivedChunks;
    std::mutex packetsMutex;

    // Packet queues
    std::queue<std::vector<char>> sendQueue;
    std::mutex sendQueueMutex;
    std::condition_variable sendQueueCV;

    // Threads
    std::unique_ptr<std::thread> senderThread;
    std::unique_ptr<std::thread> receiverThread;
    std::unique_ptr<std::thread> retransmitThread;
    std::unique_ptr<std::thread> cleanupThread;
    std::unique_ptr<std::thread> adaptiveControlThread;

    // Callback
    DataCallback dataCallback;
    std::mutex callbackMutex;

    // Metrics and adaptive control
    NetworkMetrics metrics;
    AdaptiveNetworkControl adaptiveControl;
    std::chrono::steady_clock::time_point lastMetricsUpdate;
    static constexpr std::chrono::milliseconds METRICS_UPDATE_INTERVAL{100}; // 100ms
    static constexpr std::chrono::milliseconds ADAPTIVE_CONTROL_INTERVAL{500}; // 500ms

    // Thread functions
    void senderLoop();
    void receiverLoop();
    void retransmitLoop();
    void cleanupLoop();
    void adaptiveControlLoop();

    // Helper functions
    void handleIncomingPacket(const char* data, size_t length);
    void handleAck(const AckPacket* ack);
    void handleRetransmitRequest(const RetransmitRequest* req);
    void sendAck(uint32_t sequenceNumber);
    void retransmitPacket(uint32_t sequenceNumber, uint32_t chunkIndex);
    void cleanupOldPackets();
    
    // Chunking helpers
    std::vector<std::vector<char>> chunkData(const std::vector<char>& data, PacketType type);
    bool tryReassembleMessage(uint32_t sequenceNumber);

    // Utility functions
    uint32_t getAndIncrementSequence();
    static uint32_t getCurrentTimestamp();
    
    // Metrics helpers
    void updateMetrics();
    void recordPacketMetrics(size_t bytes, bool sent);
    void recordRTT(uint32_t sequenceNumber);
    
    // Rate control helpers
    void applyRateControl();
    bool shouldSendPacket() const;
    std::chrono::microseconds calculatePacketDelay() const;
}; 

} // namespace hologram 