#include "network/NetworkManager.hpp"
#include "network/DecompressionManager.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <numeric>
#include <map>

#define CHUNK_HEADER_SIZE 8   // 4 bytes for total chunks, 4 bytes for chunk index

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

    // Set receive buffer size
    int rcvbuf = 1024 * 1024; // 1MB buffer
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
        std::cerr << "Warning: Could not set socket receive buffer size" << std::endl;
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
    std::map<uint32_t, std::vector<char>> frameBuffers;  // Map of frame chunks
    std::map<uint32_t, uint32_t> expectedChunks;        // Map of expected chunk counts
    
    while (!shouldStop) {
        ssize_t bytesReceived = recv(sockfd, buffer, sizeof(buffer), 0);
        if (bytesReceived > 0) {
            updateBandwidth(bytesReceived);
            updateRTT();

            if (bytesReceived > CHUNK_HEADER_SIZE) {
                // Extract header information
                uint32_t totalChunks = *reinterpret_cast<uint32_t*>(buffer);
                uint32_t chunkIndex = *reinterpret_cast<uint32_t*>(buffer + sizeof(uint32_t));
                
                // Generate a unique frame ID based on the first chunk received
                uint32_t frameId = (chunkIndex == 0) ? 
                    static_cast<uint32_t>(std::chrono::steady_clock::now().time_since_epoch().count()) : 
                    expectedChunks.rbegin()->first;

                // Store expected chunk count for this frame
                if (expectedChunks.find(frameId) == expectedChunks.end()) {
                    expectedChunks[frameId] = totalChunks;
                    frameBuffers[frameId].reserve(65507 * totalChunks); // Pre-allocate approximate size
                }

                // Store chunk data
                size_t dataSize = bytesReceived - CHUNK_HEADER_SIZE;
                size_t offset = chunkIndex * (65507 - CHUNK_HEADER_SIZE);
                
                // Ensure buffer is large enough
                if (frameBuffers[frameId].size() < offset + dataSize) {
                    frameBuffers[frameId].resize(offset + dataSize);
                }
                
                // Copy chunk data
                memcpy(frameBuffers[frameId].data() + offset, 
                       buffer + CHUNK_HEADER_SIZE, 
                       dataSize);

                // Check if we have all chunks for this frame
                if (frameBuffers[frameId].size() >= 
                    (expectedChunks[frameId] * (65507 - CHUNK_HEADER_SIZE))) {
                    // Pass the complete frame to DecompressionManager
                    decompManager->addCompressedFrame(std::move(frameBuffers[frameId]));
                    
                    // Cleanup
                    frameBuffers.erase(frameId);
                    expectedChunks.erase(frameId);
                }
            }
        }
    }
} 