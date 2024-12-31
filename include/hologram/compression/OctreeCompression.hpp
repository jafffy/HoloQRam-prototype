#pragma once

#include "hologram/compression/CompressionScheme.hpp"
#include <pcl/compression/octree_pointcloud_compression.h>

class OctreeCompression : public CompressionScheme {
public:
    OctreeCompression();
    
    void compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) override;
    void decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) override;
    
private:
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> compressor;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> decompressor;
}; 