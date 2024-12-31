#pragma once

#include "hologram/compression/CompressionScheme.hpp"
#include "hologram/server/CellManager.hpp"
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <netinet/in.h>
#include <glm/glm.hpp>
#include <unordered_map>

namespace hologram {

class VolumetricServer {
public:
    VolumetricServer(const std::string& compressionScheme, float cellSize, bool verbose = true);
    ~VolumetricServer();

    void start();
    void stop();

private:
    bool running;
    bool verbose;
    std::unique_ptr<hologram::CompressionScheme> compressor;
    std::unique_ptr<CellManager> cellManager;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    
    // Network socket
    int sockfd;
    struct sockaddr_in serverAddr;
    std::unordered_map<std::string, ViewportData> clientViewports;
    
    // Server thread
    std::unique_ptr<std::thread> serverThread;
    void serverLoop();
    
    // Helper functions
    void handleViewportUpdate(const char* data, size_t length, const struct sockaddr_in& clientAddr);
    void streamVisibleCells(const ViewportData& viewport, const struct sockaddr_in& clientAddr);
    void sendCell(const Cell& cell, const struct sockaddr_in& clientAddr);
    std::string getClientKey(const struct sockaddr_in& addr) const;
    void predictViewport(ViewportData& viewport) const;
    void setupSocket();
    void generateSamplePointCloud();
};

} // namespace hologram 