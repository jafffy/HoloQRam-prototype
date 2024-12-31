#include "hologram/network/BaseClient.hpp"
#include "hologram/network/NetworkManager.hpp"
#include "hologram/network/DecompressionManager.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

namespace hologram {

BaseClient::BaseClient(const std::string& compressionScheme)
    : shouldStop(false)
    , compressionScheme(compressionScheme) {
}

BaseClient::~BaseClient() {
    cleanupNetworking();
}

void BaseClient::initializeNetworking() {
    try {
        decompressionManager = std::make_unique<DecompressionManager>(compressionScheme);
        networkManager = std::make_unique<NetworkManager>(decompressionManager.get());
    }
    catch (const std::exception& e) {
        networkManager.reset();
        decompressionManager.reset();
        throw;
    }
}

void BaseClient::cleanupNetworking() {
    shouldStop = true;
    
    if (networkManager) {
        networkManager->stop();
    }
    if (decompressionManager) {
        decompressionManager->stop();
    }
    
    networkManager.reset();
    decompressionManager.reset();
}

bool BaseClient::getNextFrame(std::vector<float>& currentVertices) {
    if (!decompressionManager) {
        return false;
    }

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

} // namespace hologram 