#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <netinet/in.h>

class VolumetricServer {
public:
    VolumetricServer();
    void run();

private:
    int sockfd;
    struct sockaddr_in serverAddr, clientAddr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> compressor;

    void setupSocket();
    void generateSamplePointCloud();
    void streamPointCloud();
}; 