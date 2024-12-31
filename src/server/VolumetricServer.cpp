#include "hologram/server/VolumetricServer.hpp"
#include "hologram/compression/ViVoCompression.hpp"
#include "hologram/network/NetworkProtocol.hpp"
#include "hologram/server/CellManager.hpp"

#include <pcl/io/pcd_io.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <sstream>
#include <cstring>

#define PORT 8765
#define MAX_PACKET_SIZE 65507 // Max UDP packet size
#define CHUNK_HEADER_SIZE 16   // 4 bytes each for: total chunks, chunk index, packet type, cell ID

namespace hologram {

VolumetricServer::VolumetricServer(const std::string& compressionScheme, float cellSize) 
    : running(false)
    , compressor(CompressionScheme::create(compressionScheme))
    , cellManager(std::make_unique<CellManager>(cellSize)) {
    setupSocket();
    generateSamplePointCloud();
    cellManager->segmentPointCloud(cloud);
}

VolumetricServer::~VolumetricServer() {
    stop();
    close(sockfd);
}

void VolumetricServer::start() {
    if (running) return;
    running = true;
    serverThread = std::make_unique<std::thread>(&VolumetricServer::serverLoop, this);
}

void VolumetricServer::stop() {
    if (!running) return;
    running = false;
    if (serverThread && serverThread->joinable()) {
        serverThread->join();
    }
}

void VolumetricServer::serverLoop() {
    char buffer[MAX_PACKET_SIZE];
    struct sockaddr_in clientAddr;
    socklen_t clientLen = sizeof(clientAddr);

    while (running) {
        // Check for incoming viewport updates
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 1000; // 1ms timeout

        if (select(sockfd + 1, &readfds, NULL, NULL, &tv) > 0) {
            ssize_t recvLen = recvfrom(sockfd, buffer, MAX_PACKET_SIZE, 0,
                                     (struct sockaddr*)&clientAddr, &clientLen);
            
            if (recvLen > 0) {
                PacketType type = static_cast<PacketType>(buffer[0]);
                if (type == PacketType::VIEWPORT_UPDATE) {
                    handleViewportUpdate(buffer + 1, recvLen - 1, clientAddr);
                }
            }
        }

        // Update viewports with prediction and stream visible cells
        for (auto& [key, viewport] : clientViewports) {
            predictViewport(viewport);
            
            struct sockaddr_in clientAddr;
            std::string ip = key.substr(0, key.find(':'));
            int port = std::stoi(key.substr(key.find(':') + 1));
            
            clientAddr.sin_family = AF_INET;
            clientAddr.sin_port = htons(port);
            inet_pton(AF_INET, ip.c_str(), &clientAddr.sin_addr);
            
            streamVisibleCells(viewport, clientAddr);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
}

void VolumetricServer::handleViewportUpdate(const char* data, size_t length, const struct sockaddr_in& clientAddr) {
    if (length < sizeof(ViewportData)) return;

    ViewportData viewport;
    std::memcpy(&viewport, data, sizeof(ViewportData));
    
    std::string clientKey = getClientKey(clientAddr);
    clientViewports[clientKey] = viewport;

    // Update cell visibility for the new viewport
    cellManager->updateCellVisibility(viewport);
}

void VolumetricServer::streamVisibleCells(const ViewportData& viewport, const struct sockaddr_in& clientAddr) {
    auto visibleCells = cellManager->getVisibleCells(viewport);
    
    // Stream cells in order of importance
    for (const auto* cell : visibleCells) {
        sendCell(*cell, clientAddr);
    }
}

void VolumetricServer::sendCell(const Cell& cell, const struct sockaddr_in& clientAddr) {
    // Compress cell data
    std::vector<char> compressedData;
    compressor->compress(cell.points, compressedData);
    
    // Split compressed data into chunks
    size_t totalChunks = (compressedData.size() + MAX_PACKET_SIZE - CHUNK_HEADER_SIZE - 1) / 
                        (MAX_PACKET_SIZE - CHUNK_HEADER_SIZE);
    
    static uint32_t cellIdCounter = 0;
    uint32_t cellId = cellIdCounter++;
    
    std::vector<char> packet(MAX_PACKET_SIZE);
    for (size_t i = 0; i < totalChunks; ++i) {
        // Prepare packet header
        packet[0] = static_cast<char>(PacketType::POINT_CLOUD_DATA);
        *reinterpret_cast<uint32_t*>(&packet[1]) = totalChunks;
        *reinterpret_cast<uint32_t*>(&packet[5]) = i;
        *reinterpret_cast<uint32_t*>(&packet[9]) = cellId;
        
        // Calculate chunk size
        size_t chunkStart = i * (MAX_PACKET_SIZE - CHUNK_HEADER_SIZE);
        size_t chunkSize = std::min(static_cast<size_t>(MAX_PACKET_SIZE - CHUNK_HEADER_SIZE),
                                  compressedData.size() - chunkStart);
        
        // Copy chunk data
        std::memcpy(&packet[CHUNK_HEADER_SIZE], &compressedData[chunkStart], chunkSize);
        
        // Send to client
        sendto(sockfd, packet.data(), CHUNK_HEADER_SIZE + chunkSize, 0,
               (struct sockaddr*)&clientAddr, sizeof(clientAddr));
        
        // Small delay to prevent network congestion
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

std::string VolumetricServer::getClientKey(const struct sockaddr_in& addr) const {
    char ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);
    return std::string(ip) + ":" + std::to_string(ntohs(addr.sin_port));
}

void VolumetricServer::predictViewport(ViewportData& viewport) const {
    // Simple linear prediction based on previous position
    // In a real implementation, this would use more sophisticated ML models
    static const float PREDICTION_WINDOW = 0.1f; // 100ms prediction
    
    // For now, just keep the current viewport
    // TODO: Implement ML-based prediction
}

void VolumetricServer::setupSocket() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        exit(1);
    }

    // Enable socket buffer size option
    int sendbuff = 1024 * 1024; // 1MB buffer
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sendbuff, sizeof(sendbuff)) < 0) {
        std::cerr << "Warning: Could not set socket buffer size" << std::endl;
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(PORT);

    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        exit(1);
    }
}

void VolumetricServer::generateSamplePointCloud() {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // TODO: Load actual point cloud data
    // For now, just create a simple cube of points
    for (float x = -1; x <= 1; x += 0.1) {
        for (float y = -1; y <= 1; y += 0.1) {
            for (float z = -1; z <= 1; z += 0.1) {
                pcl::PointXYZRGB point;
                point.x = x * 100; // Scale to 100cm
                point.y = y * 100;
                point.z = z * 100;
                point.r = static_cast<uint8_t>((x + 1) * 127.5);
                point.g = static_cast<uint8_t>((y + 1) * 127.5);
                point.b = static_cast<uint8_t>((z + 1) * 127.5);
                cloud->push_back(point);
            }
        }
    }
}

} // namespace hologram 