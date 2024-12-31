#pragma once

#include <string>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <netinet/in.h>
#include "hologram/compression/CompressionScheme.hpp"

class VolumetricServer {
public:
    VolumetricServer(const std::string& compressionScheme = "octree");
    void run();

private:
    int sockfd;
    struct sockaddr_in serverAddr, clientAddr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    std::unique_ptr<CompressionScheme> compressor;

    void setupSocket();
    void generateSamplePointCloud();
    void streamPointCloud();
}; 