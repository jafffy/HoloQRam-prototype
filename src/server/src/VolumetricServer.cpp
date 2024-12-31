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
#define BUFFER_SIZE 65507 // Max UDP packet size

VolumetricServer::VolumetricServer() : compressor(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false) {
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
        std::stringstream compressedData;
        compressor.encodePointCloud(cloud, compressedData);
        
        std::string data = compressedData.str();
        size_t totalSize = data.size();

        // First send the size of the data
        clientAddr.sin_family = AF_INET;
        clientAddr.sin_port = htons(8766);
        inet_pton(AF_INET, "127.0.0.1", &clientAddr.sin_addr);

        uint32_t size = static_cast<uint32_t>(totalSize);
        sendto(sockfd, &size, sizeof(size), 0,
                (struct sockaddr*)&clientAddr, sizeof(clientAddr));

        // Then send the data in chunks
        size_t offset = 0;
        while (offset < totalSize) {
            size_t chunkSize = std::min(BUFFER_SIZE - sizeof(uint32_t), totalSize - offset);
            std::string chunk = data.substr(offset, chunkSize);
            
            sendto(sockfd, chunk.c_str(), chunk.size(), 0,
                    (struct sockaddr*)&clientAddr, sizeof(clientAddr));
            
            offset += chunkSize;
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Small delay between chunks
        }
    } catch (const std::exception& e) {
        std::cerr << "Error streaming point cloud: " << e.what() << std::endl;
    }
} 