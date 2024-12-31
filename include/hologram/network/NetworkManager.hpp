#pragma once

#include <netinet/in.h>
#include <thread>
#include <atomic>
#include <deque>
#include <mutex>
#include <chrono>

class DecompressionManager;

class NetworkManager {
public:
    NetworkManager(DecompressionManager* decompManager);
    ~NetworkManager();
    
    void start();
    void stop();
    
    double getCurrentBandwidth() const { return currentBandwidth; }
    double getCurrentRTT() const { return currentRTT; }

private:
    void setupSocket();
    void receiveData();
    void updateBandwidth(size_t newBytes);
    void updateRTT();
    
    DecompressionManager* decompManager;  // Non-owning pointer
    int sockfd;
    struct sockaddr_in serverAddr;
    std::thread receiveThread;
    
    std::atomic<uint64_t> totalBytesReceived;
    std::chrono::steady_clock::time_point lastBandwidthCheck;
    std::chrono::steady_clock::time_point lastPacketTime;
    std::atomic<double> currentBandwidth;
    std::atomic<double> currentRTT;
    
    std::mutex rttMutex;
    std::deque<double> rttHistory;
    static constexpr size_t RTT_HISTORY_SIZE = 10;
    
    std::atomic<bool> shouldStop;
}; 