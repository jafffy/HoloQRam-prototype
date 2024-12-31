#include "hologram/network/NetworkManager.hpp"
#include "hologram/network/DecompressionManager.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <numeric>
#include <map>
#include <fcntl.h>
#include <thread>

using namespace hologram;  // For PacketType enum

NetworkManager::NetworkManager(DecompressionManager* decompManager,
                             const std::string& serverIP,
                             int serverPort)
    : decompManager(decompManager)
    , serverIP(serverIP)
    , serverPort(serverPort)
    , sockfd(-1)
    , totalBytesReceived(0)
    , totalBytesSent(0)
    , lastBandwidthCheck(std::chrono::steady_clock::now())
    , lastPacketTime(std::chrono::steady_clock::now())
    , currentBandwidth(0.0)
    , currentRTT(0.0)
    , shouldStop(false)
{
    if (!decompManager) {
        throw std::runtime_error("DecompressionManager pointer cannot be null");
    }
    setupSocket();  // Only setup the socket, don't start receiving yet
}

NetworkManager::~NetworkManager() {
    stop();
    if (sockfd >= 0) {
        close(sockfd);
    }
}

void NetworkManager::setupSocket() {
    // Close existing socket if any
    if (sockfd >= 0) {
        std::cout << "[NetworkManager::setupSocket] Closing existing socket..." << std::endl;
        close(sockfd);
        sockfd = -1;
    }

    std::cout << "[NetworkManager::setupSocket] Creating socket..." << std::endl;
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "[NetworkManager::setupSocket] Error creating socket: " << strerror(errno) << std::endl;
        throw std::runtime_error("Error creating socket");
    }
    std::cout << "[NetworkManager::setupSocket] Socket created successfully with fd: " << sockfd << std::endl;

    // Set receive buffer size
    int rcvbuf = 1024 * 1024; // 1MB buffer
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
        std::cerr << "[NetworkManager::setupSocket] Warning: Could not set socket receive buffer size: " << strerror(errno) << std::endl;
    } else {
        std::cout << "[NetworkManager::setupSocket] Set receive buffer size to 1MB" << std::endl;
    }

    // Set socket options for reuse
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "[NetworkManager::setupSocket] Warning: Could not set SO_REUSEADDR: " << strerror(errno) << std::endl;
    }

    // Set non-blocking mode
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) {
        std::cerr << "[NetworkManager::setupSocket] Warning: Could not get socket flags: " << strerror(errno) << std::endl;
    } else {
        if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
            std::cerr << "[NetworkManager::setupSocket] Warning: Could not set non-blocking mode: " << strerror(errno) << std::endl;
        } else {
            std::cout << "[NetworkManager::setupSocket] Set socket to non-blocking mode" << std::endl;
        }
    }

    // Set up server address
    std::cout << "[NetworkManager::setupSocket] Setting up server address: " << serverIP << ":" << serverPort << std::endl;
    memset(&serverAddr, 0, sizeof(serverAddr));  // Clear the structure first
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    
    // Convert IP address from string to binary form
    if (inet_pton(AF_INET, serverIP.c_str(), &serverAddr.sin_addr) <= 0) {
        std::cerr << "[NetworkManager::setupSocket] Invalid IP address: " << serverIP << " - " << strerror(errno) << std::endl;
        throw std::runtime_error("Invalid server IP address");
    }
    std::cout << "[NetworkManager::setupSocket] Successfully configured server address" << std::endl;

    // For UDP, we don't need to connect. Instead, we'll use sendto/recvfrom
    std::cout << "[NetworkManager::setupSocket] UDP socket setup complete" << std::endl;

    // Send initial viewport update to establish communication
    PacketHeader header;
    header.type = PacketType::VIEWPORT_UPDATE;
    header.sequenceNumber = 0;
    header.timestamp = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
    header.totalChunks = 1;
    header.chunkIndex = 0;
    header.payloadSize = sizeof(ViewportInfo);

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

    std::vector<char> packet(sizeof(PacketHeader) + sizeof(ViewportInfo));
    std::memcpy(packet.data(), &header, sizeof(PacketHeader));
    std::memcpy(packet.data() + sizeof(PacketHeader), &viewport, sizeof(ViewportInfo));

    std::cout << "[NetworkManager::setupSocket] Sending initial viewport update..." << std::endl;
    ssize_t bytesSent = sendto(sockfd, packet.data(), packet.size(), 0,
                              (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    if (bytesSent < 0) {
        std::cerr << "[NetworkManager::setupSocket] Warning: Failed to send initial viewport update: " 
                  << strerror(errno) << std::endl;
    } else {
        std::cout << "[NetworkManager::setupSocket] Sent initial viewport update (" << bytesSent << " bytes)" << std::endl;
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

void NetworkManager::updateBandwidth(size_t newBytes, bool isSent) {
    if (isSent) {
        totalBytesSent += newBytes;
    } else {
        totalBytesReceived += newBytes;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastBandwidthCheck).count();
    
    // Update bandwidth every 500ms
    if (duration >= 500) {
        double seconds = duration / 1000.0;
        double totalBytes = totalBytesSent + totalBytesReceived;
        double bytesPerSecond = static_cast<double>(totalBytes) / seconds;
        currentBandwidth = bytesPerSecond / (1024.0 * 1024.0); // Convert to MB/s
        
        std::cout << "[NetworkManager::updateBandwidth] Stats - "
                  << "Duration: " << duration << "ms, "
                  << "Total bytes sent: " << totalBytesSent << ", "
                  << "Total bytes received: " << totalBytesReceived << ", "
                  << "Total bytes: " << totalBytes << ", "
                  << "Bytes/sec: " << bytesPerSecond << ", "
                  << "Current bandwidth: " << currentBandwidth << " MB/s" << std::endl;
        
        // Only reset counters after we've used them for calculation
        totalBytesSent = 0;
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
    char buffer[MAX_PACKET_SIZE];
    ssize_t bytesRead;
    int errorCount = 0;
    const int MAX_CONSECUTIVE_ERRORS = 5;
    struct sockaddr_in senderAddr;
    socklen_t senderLen = sizeof(senderAddr);
    size_t totalBytesThisInterval = 0;
    size_t packetsReceived = 0;
    size_t pointCloudPacketsReceived = 0;

    std::cout << "[NetworkManager::receiveData] Starting receive loop on socket " << sockfd << std::endl;

    while (!shouldStop) {
        bytesRead = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                            (struct sockaddr*)&senderAddr, &senderLen);
        
        if (bytesRead < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available, wait a bit and try again
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            errorCount++;
            std::cerr << "[NetworkManager::receiveData] Error receiving data: " << strerror(errno) 
                      << " (errno: " << errno << "), error count: " << errorCount 
                      << ", socket: " << sockfd << std::endl;
            
            if (errorCount >= MAX_CONSECUTIVE_ERRORS) {
                std::cerr << "[NetworkManager::receiveData] Too many consecutive errors, breaking receive loop" << std::endl;
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Reset error count on successful receive
        errorCount = 0;
        totalBytesThisInterval += bytesRead;
        packetsReceived++;

        // Process received data
        if (bytesRead >= sizeof(PacketHeader)) {
            const PacketHeader* header = reinterpret_cast<const PacketHeader*>(buffer);
            
            std::cout << "[NetworkManager::receiveData] Received packet from "
                      << inet_ntoa(senderAddr.sin_addr) << ":" << ntohs(senderAddr.sin_port) << "\n"
                      << "  Type: " << static_cast<int>(header->type) << "\n"
                      << "  Size: " << bytesRead << " bytes\n"
                      << "  Sequence: " << header->sequenceNumber << "\n"
                      << "  Total chunks: " << header->totalChunks << "\n"
                      << "  Chunk index: " << header->chunkIndex << "\n"
                      << "  Payload size: " << header->payloadSize << std::endl;

            // Update metrics with received bytes
            updateBandwidth(bytesRead, false);
            updateRTT();

            // Process based on packet type
            switch (header->type) {
                case PacketType::POINT_CLOUD_DATA: {
                    pointCloudPacketsReceived++;
                    if (bytesRead > sizeof(PacketHeader)) {
                        size_t dataSize = bytesRead - sizeof(PacketHeader);
                        uint32_t totalChunks = header->totalChunks;
                        uint32_t chunkIndex = header->chunkIndex;
                        
                        std::cout << "[NetworkManager::receiveData] Processing point cloud chunk " 
                                  << (chunkIndex + 1) << "/" << totalChunks
                                  << " (data size: " << dataSize << " bytes)\n"
                                  << "  Total point cloud packets received: " << pointCloudPacketsReceived
                                  << std::endl;
                        
                        // Pass compressed data to decompression manager
                        std::vector<char> compressedData(
                            buffer + sizeof(PacketHeader),
                            buffer + bytesRead
                        );
                        
                        try {
                            decompManager->addCompressedData(compressedData);
                            std::cout << "[NetworkManager::receiveData] Successfully added chunk to decompression queue" 
                                      << std::endl;
                        } catch (const std::exception& e) {
                            std::cerr << "[NetworkManager::receiveData] Error adding data to decompression queue: "
                                      << e.what() << std::endl;
                        }
                    }
                    break;
                }
                default:
                    std::cerr << "[NetworkManager::receiveData] Unknown packet type: " 
                              << static_cast<int>(header->type) << std::endl;
                    break;
            }
        } else {
            std::cerr << "[NetworkManager::receiveData] Received packet too small: " 
                      << bytesRead << " bytes (minimum size: " << sizeof(PacketHeader) 
                      << " bytes)" << std::endl;
        }

        // Log statistics periodically
        if (packetsReceived % 100 == 0) {
            std::cout << "[NetworkManager::receiveData] Statistics:\n"
                      << "  Total packets received: " << packetsReceived << "\n"
                      << "  Point cloud packets received: " << pointCloudPacketsReceived << "\n"
                      << "  Total bytes received: " << totalBytesThisInterval << std::endl;
        }
    }
}

bool NetworkManager::sendViewportUpdate(const ViewportInfo& viewport) {
    std::cout << "[NetworkManager::sendViewportUpdate] Preparing viewport update packet..." << std::endl;
    
    // Create packet header
    PacketHeader header;
    header.type = PacketType::VIEWPORT_UPDATE;
    header.sequenceNumber = 0;  // Not used for viewport updates
    header.timestamp = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
    header.totalChunks = 1;
    header.chunkIndex = 0;
    header.payloadSize = sizeof(ViewportInfo);

    std::cout << "[NetworkManager::sendViewportUpdate] Viewport data - "
              << "Position: [" << viewport.position[0] << ", " << viewport.position[1] << ", " << viewport.position[2] << "], "
              << "Rotation: [" << viewport.rotation[0] << ", " << viewport.rotation[1] << ", " << viewport.rotation[2] << "]" << std::endl;

    // Combine header and viewport data
    std::vector<char> packet(sizeof(PacketHeader) + sizeof(ViewportInfo));
    std::memcpy(packet.data(), &header, sizeof(PacketHeader));
    std::memcpy(packet.data() + sizeof(PacketHeader), &viewport, sizeof(ViewportInfo));

    // Send packet using sendto instead of send
    std::cout << "[NetworkManager::sendViewportUpdate] Attempting to send packet of size " << packet.size() << " bytes..." << std::endl;
    ssize_t bytesSent = sendto(sockfd, packet.data(), packet.size(), 0,
                              (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    if (bytesSent < 0) {
        std::cerr << "[NetworkManager::sendViewportUpdate] Failed to send viewport update: " 
                  << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        return false;
    }

    // Update bandwidth with sent bytes
    updateBandwidth(bytesSent, true);

    std::cout << "[NetworkManager::sendViewportUpdate] Successfully sent viewport update (" 
              << bytesSent << " bytes)" << std::endl;
    return true;
}

void NetworkManager::updateServerDetails(const std::string& newServerIP, int newServerPort) {
    // Only update if details have changed
    if (serverIP == newServerIP && serverPort == newServerPort) {
        return;
    }
    
    // Store new details
    serverIP = newServerIP;
    serverPort = newServerPort;
    
    // Stop current connection if any
    stop();
    if (sockfd >= 0) {
        close(sockfd);
        sockfd = -1;
    }
    
    // Setup new connection
    setupSocket();
    start();
} 