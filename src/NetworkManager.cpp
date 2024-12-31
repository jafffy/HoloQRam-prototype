#include "VolumetricClient.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <numeric>

NetworkManager::NetworkManager(DecompressionManager* decompManager)
    : decompManager(decompManager)
    , sockfd(-1)
    , totalBytesReceived(0)
    , lastBandwidthCheck(std::chrono::steady_clock::now())
    , lastPacketTime(std::chrono::steady_clock::now())
    , currentBandwidth(0.0)
    , currentRTT(0.0)
    , shouldStop(false)
{
    if (!decompManager) {
        throw std::runtime_error("DecompressionManager pointer cannot be null");
    }
    setupSocket();
}

NetworkManager::~NetworkManager() {
    stop();
    if (sockfd >= 0) {
        close(sockfd);
    }
}

void NetworkManager::setupSocket() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        throw std::runtime_error("Error creating socket");
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8766);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        close(sockfd);
        throw std::runtime_error("Error binding socket");
    }
}

void NetworkManager::start() {
    shouldStop = false;
    receiveThread = std::thread(&NetworkManager::receiveData, this);
}

void NetworkManager::stop() {
    shouldStop = true;
    if (receiveThread.joinable()) {
        receiveThread.join();
    }
}

void NetworkManager::updateBandwidth(size_t newBytes) {
    totalBytesReceived += newBytes;
    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastBandwidthCheck).count();
    
    // Update bandwidth every 500ms
    if (duration >= 500) {
        double seconds = duration / 1000.0;
        double bytesPerSecond = static_cast<double>(totalBytesReceived) / seconds;
        currentBandwidth = bytesPerSecond / (1024.0 * 1024.0); // Convert to MB/s
        
        totalBytesReceived = 0;
        lastBandwidthCheck = now;
    }
}

void NetworkManager::updateRTT() {
    auto now = std::chrono::steady_clock::now();
    double rtt = std::chrono::duration_cast<std::chrono::microseconds>(
        now - lastPacketTime).count() / 1000.0; // Convert to milliseconds

    std::lock_guard<std::mutex> lock(rttMutex);
    rttHistory.push_back(rtt);
    if (rttHistory.size() > RTT_HISTORY_SIZE) {
        rttHistory.pop_front();
    }
    
    // Calculate average RTT
    currentRTT = std::accumulate(rttHistory.begin(), rttHistory.end(), 0.0) / rttHistory.size();
    
    // Update last packet time
    lastPacketTime = now;
}

void NetworkManager::receiveData() {
    char buffer[65507];  // Maximum UDP packet size
    std::vector<char> receivedData;
    uint32_t expectedSize = 0;
    bool waitingForSize = true;
    
    while (!shouldStop) {
        ssize_t bytesReceived = recv(sockfd, buffer, sizeof(buffer), 0);
        if (bytesReceived > 0) {
            updateBandwidth(bytesReceived);
            updateRTT();

            if (waitingForSize) {
                if (bytesReceived >= sizeof(uint32_t)) {
                    expectedSize = *reinterpret_cast<uint32_t*>(buffer);
                    receivedData.reserve(expectedSize);
                    waitingForSize = false;
                }
            } else {
                receivedData.insert(receivedData.end(), buffer, buffer + bytesReceived);
                
                if (receivedData.size() >= expectedSize) {
                    // Pass the received frame to DecompressionManager
                    decompManager->addCompressedFrame(std::move(receivedData));
                    
                    // Reset for next frame
                    receivedData = std::vector<char>();
                    waitingForSize = true;
                }
            }
        }
    }
} 