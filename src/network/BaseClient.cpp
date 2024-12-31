#include "network/BaseClient.hpp"
#include "network/NetworkManager.hpp"
#include "network/DecompressionManager.hpp"

#include <iostream>

BaseClient::BaseClient(const std::string& compressionScheme)
    : shouldStop(false)
    , compressionScheme(compressionScheme) {
    initializeNetworking();
}

BaseClient::~BaseClient() {
    cleanupNetworking();
}

void BaseClient::initializeNetworking() {
    try {
        // Initialize components in the correct order
        decompressionManager = std::make_unique<DecompressionManager>(compressionScheme);
        networkManager = std::make_unique<NetworkManager>(decompressionManager.get());

        // Start network and decompression threads
        decompressionManager->start();  // Start decompression first
        networkManager->start();        // Then start network to receive frames
    }
    catch (const std::exception& e) {
        // Clean up in case of initialization failure
        networkManager.reset();
        decompressionManager.reset();
        throw;
    }
}

void BaseClient::cleanupNetworking() {
    shouldStop = true;
    networkManager.reset();
    decompressionManager.reset();
}

bool BaseClient::getNextFrame(std::vector<float>& currentVertices) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    if (!decompressionManager->getDecompressedCloud(cloud)) {
        return false;
    }

    // Convert point cloud to vertices
    currentVertices.clear();
    currentVertices.reserve(cloud->points.size() * 3);
    for (const auto& point : cloud->points) {
        currentVertices.push_back(point.x);
        currentVertices.push_back(point.y);
        currentVertices.push_back(point.z);
    }
    return true;
} 