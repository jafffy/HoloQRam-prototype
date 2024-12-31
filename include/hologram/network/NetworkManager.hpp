#pragma once

// System includes
#include <netinet/in.h>

// Standard library includes
#include <thread>
#include <atomic>
#include <deque>
#include <mutex>
#include <chrono>
#include <string>

// Project includes
#include "hologram/network/NetworkProtocol.hpp"

// Forward declarations
namespace hologram {
    class DecompressionManager;
}

namespace hologram {

class NetworkManager {
public:
    NetworkManager(DecompressionManager* decompManager,
                  const std::string& serverIP = "127.0.0.1",
                  int serverPort = 8765);
    ~NetworkManager();
    
    void start();
    void stop();
    void updateServerDetails(const std::string& newServerIP, int newServerPort);
    bool sendViewportUpdate(const ViewportInfo& viewport);
    
    double getCurrentBandwidth() const { return currentBandwidth; }
    double getCurrentRTT() const { return currentRTT; }

private:
    void setupSocket();
    void receiveData();
    void updateBandwidth(size_t newBytes, bool isSent);
    void updateRTT();
    
    DecompressionManager* decompManager;  // Non-owning pointer
    std::string serverIP;
    int serverPort;
    int sockfd;
    struct sockaddr_in serverAddr;
    std::thread receiveThread;
    
    std::atomic<uint64_t> totalBytesReceived{0};
    std::atomic<uint64_t> totalBytesSent{0};
    std::chrono::steady_clock::time_point lastBandwidthCheck;
    std::chrono::steady_clock::time_point lastPacketTime;
    std::atomic<double> currentBandwidth{0.0};
    std::atomic<double> currentRTT{0.0};
    
    std::mutex rttMutex;
    std::deque<double> rttHistory;
    static constexpr size_t RTT_HISTORY_SIZE = 10;
    
    std::atomic<bool> shouldStop;
};

} // namespace hologram 