#include "server/VolumetricServer.hpp"

#include <pcl/io/pcd_io.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <sstream>

#define PORT 8765
#define MAX_PACKET_SIZE 65507 // Max UDP packet size
#define CHUNK_HEADER_SIZE 8   // 4 bytes for total chunks, 4 bytes for chunk index

VolumetricServer::VolumetricServer(const std::string& compressionScheme) 
    : compressor(CompressionScheme::create(compressionScheme)) {
    setupSocket();
    generateSamplePointCloud();
}

void VolumetricServer::run() {
    while (true) {
        streamPointCloud();
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
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

    std::cout << "Server started on port " << PORT << std::endl;
}

void VolumetricServer::generateSamplePointCloud() {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Generate random points
    for (int i = 0; i < 1000; ++i) {
        pcl::PointXYZRGB point;
        point.x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        point.y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        point.z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        
        // Random RGB colors
        point.r = rand() % 255;
        point.g = rand() % 255;
        point.b = rand() % 255;
        
        cloud->points.push_back(point);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
}

void VolumetricServer::streamPointCloud() {
    try {
        std::vector<char> compressedData;
        compressor->compress(cloud, compressedData);
        
        size_t totalSize = compressedData.size();
        
        // Calculate number of chunks needed
        size_t maxChunkDataSize = MAX_PACKET_SIZE - CHUNK_HEADER_SIZE;
        uint32_t totalChunks = (totalSize + maxChunkDataSize - 1) / maxChunkDataSize;

        // Set up client address
        clientAddr.sin_family = AF_INET;
        clientAddr.sin_port = htons(8766);
        inet_pton(AF_INET, "127.0.0.1", &clientAddr.sin_addr);

        // Send data in chunks
        for (uint32_t chunkIndex = 0; chunkIndex < totalChunks; ++chunkIndex) {
            size_t offset = chunkIndex * maxChunkDataSize;
            size_t chunkSize = std::min(maxChunkDataSize, totalSize - offset);
            
            // Prepare packet with header
            std::vector<char> packet(CHUNK_HEADER_SIZE + chunkSize);
            memcpy(packet.data(), &totalChunks, sizeof(totalChunks));
            memcpy(packet.data() + sizeof(totalChunks), &chunkIndex, sizeof(chunkIndex));
            memcpy(packet.data() + CHUNK_HEADER_SIZE, compressedData.data() + offset, chunkSize);
            
            // Send packet
            sendto(sockfd, packet.data(), packet.size(), 0,
                   (struct sockaddr*)&clientAddr, sizeof(clientAddr));
            
            // Small delay between chunks to prevent overwhelming the network
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    } catch (const std::exception& e) {
        std::cerr << "Error streaming point cloud: " << e.what() << std::endl;
    }
} 