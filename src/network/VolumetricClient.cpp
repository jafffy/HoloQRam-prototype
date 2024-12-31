#include "hologram/network/VolumetricClient.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <thread>
#include <iomanip>

namespace hologram {

VolumetricClient::VolumetricClient(const std::string& serverIP, int serverPort)
    : BaseClient("vivo")  // Using ViVo compression by default
    , running(false)
    , sockfd(-1)
    , frameCount(0)
    , currentFPS(0.0)
{
    std::cout << "[VolumetricClient] Initializing with server " << serverIP << ":" << serverPort << std::endl;
    
    // Initialize socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        throw std::runtime_error("Error creating socket");
    }
    
    // Set socket buffer size
    int recvbuff = 1024 * 1024; // 1MB buffer
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &recvbuff, sizeof(recvbuff)) < 0) {
        std::cerr << "[VolumetricClient] Warning: Could not set socket buffer size" << std::endl;
    }

    // Initialize network manager
    networkManager = std::make_unique<ReliableNetworkManager>(sockfd);
}

VolumetricClient::~VolumetricClient() {
    stop();
    if (sockfd >= 0) {
        close(sockfd);
        sockfd = -1;
    }
}

void VolumetricClient::run() {
    start();
    while (!shouldStop) {
        clientLoop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    stop();
}

void VolumetricClient::start() {
    if (running) return;
    
    std::cout << "[VolumetricClient] Starting client..." << std::endl;
    running = true;
    networkManager->start();
    clientThread = std::make_unique<std::thread>(&VolumetricClient::clientLoop, this);
    std::cout << "[VolumetricClient] Client started successfully" << std::endl;
}

void VolumetricClient::stop() {
    if (!running) return;
    
    running = false;
    networkManager->stop();
    
    if (clientThread && clientThread->joinable()) {
        clientThread->join();
    }
}

void VolumetricClient::clearMetricsBuffer() {
    metricsBuffer.str("");
    metricsBuffer.clear();
}

void VolumetricClient::updatePerformanceMetrics() {
    auto currentTime = std::chrono::steady_clock::now();
    frameCount++;

    // Update FPS every second
    auto timeSinceLastUpdate = std::chrono::duration<double>(currentTime - lastFPSUpdate).count();
    if (timeSinceLastUpdate >= 1.0) {
        currentFPS = frameCount / timeSinceLastUpdate;
        frameCount = 0;
        lastFPSUpdate = currentTime;
    }
}

void VolumetricClient::displayMetrics() {
    auto currentTime = std::chrono::steady_clock::now();
    auto timeSinceLastDisplay = std::chrono::duration<double>(currentTime - lastMetricsDisplay).count();
    
    // Only update display at specified interval
    if (timeSinceLastDisplay < METRICS_UPDATE_INTERVAL) {
        return;
    }
    
    // Clear previous buffer content
    clearMetricsBuffer();

    // Get network metrics
    auto packetMetrics = networkManager->getPacketMetrics();
    auto compressionMetrics = networkManager->getCompressionMetrics();
    auto networkParams = networkManager->getNetworkParams();
    
    // Build metrics string in buffer
    metricsBuffer << "\033[2J\033[H"  // Clear screen and move cursor to top
                 << "Performance Metrics:\n"
                 << "==================\n"
                 << "FPS: " << currentFPS << "\n"
                 << "Network Metrics:\n"
                 << "  RTT: " << packetMetrics.averageRoundTripTime << " ms\n"
                 << "  Packet Loss: " << (packetMetrics.packetLossRate * 100.0f) << "%\n"
                 << "  Bandwidth: " << (packetMetrics.currentBandwidth / 1024.0f) << " KB/s\n"
                 << "  Packets: " << packetMetrics.packetsSent << " sent, "
                 << packetMetrics.packetsReceived << " received, "
                 << packetMetrics.packetsLost << " lost\n"
                 << "Compression Metrics:\n"
                 << "  Ratio: " << compressionMetrics.averageCompressionRatio << "\n"
                 << "  Time: " << compressionMetrics.averageCompressionTime << " ms compress, "
                 << compressionMetrics.averageDecompressionTime << " ms decompress\n"
                 << "  Total: " << (compressionMetrics.totalCompressedBytes / 1024.0f) << " KB compressed, "
                 << (compressionMetrics.totalUncompressedBytes / 1024.0f) << " KB uncompressed\n"
                 << "\nPress Enter to stop...\n";
    
    // Write buffer to output and flush
    std::cout << metricsBuffer.str() << std::flush;
    
    lastMetricsDisplay = currentTime;
}

void VolumetricClient::clientLoop() {
    std::cout << "[VolumetricClient] Entering client loop" << std::endl;
    int frameCounter = 0;
    
    // Pre-allocate packet buffer
    std::vector<char> packetBuffer;
    packetBuffer.reserve(sizeof(PacketHeader) + sizeof(ViewportInfo));
    
    while (running) {
        try {
            // Create packet header
            PacketHeader header;
            header.type = PacketType::VIEWPORT_UPDATE;
            header.sequenceNumber = 0;  // Network manager will set this
            header.timestamp = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );
            header.totalChunks = 1;
            header.chunkIndex = 0;
            header.payloadSize = sizeof(ViewportInfo);

            // Create viewport data
            ViewportInfo viewport;
            viewport.position[0] = 0.0f;
            viewport.position[1] = 0.0f;
            viewport.position[2] = 0.0f;
            viewport.rotation[0] = 0.0f;
            viewport.rotation[1] = 0.0f;
            viewport.rotation[2] = 0.0f;
            viewport.fov = 60.0f;
            viewport.aspectRatio = 16.0f / 9.0f;
            viewport.nearPlane = 0.1f;
            viewport.farPlane = 100.0f;

            // Combine header and viewport data
            packetBuffer.clear();
            packetBuffer.resize(sizeof(PacketHeader) + sizeof(ViewportInfo));
            std::memcpy(packetBuffer.data(), &header, sizeof(PacketHeader));
            std::memcpy(packetBuffer.data() + sizeof(PacketHeader), &viewport, sizeof(ViewportInfo));
            
            // Send via network manager
            std::cout << "[VolumetricClient] Attempting to send frame " << frameCounter 
                     << " (size: " << packetBuffer.size() << " bytes)" << std::endl;
            
            bool sendResult = false;
            try {
                sendResult = networkManager->sendReliable(packetBuffer, PacketType::VIEWPORT_UPDATE);
            } catch (const std::exception& e) {
                std::cerr << "[VolumetricClient] Exception during send: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (sendResult) {
                frameCounter++;
                if (frameCounter % 30 == 0) {
                    std::cout << "[VolumetricClient] Successfully sent frame " << frameCounter << std::endl;
                    
                    // Log current metrics
                    auto packetMetrics = networkManager->getPacketMetrics();
                    std::cout << "[VolumetricClient] Current metrics:"
                              << "\n  Packets sent: " << packetMetrics.packetsSent
                              << "\n  Packets received: " << packetMetrics.packetsReceived
                              << "\n  RTT: " << packetMetrics.averageRoundTripTime << " ms"
                              << std::endl;
                }

                // Update and display metrics
                try {
                    updatePerformanceMetrics();
                    displayMetrics();
                } catch (const std::exception& e) {
                    std::cerr << "[VolumetricClient] Exception during metrics update: " << e.what() << std::endl;
                }
            } else {
                std::cerr << "[VolumetricClient] Failed to send frame " << frameCounter << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Sleep for a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        } catch (const std::exception& e) {
            std::cerr << "[VolumetricClient] Error in client loop: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait a bit before retrying
        } catch (...) {
            std::cerr << "[VolumetricClient] Unknown error in client loop" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait a bit before retrying
        }
    }
    std::cout << "[VolumetricClient] Client loop ended" << std::endl;
}

} // namespace hologram 