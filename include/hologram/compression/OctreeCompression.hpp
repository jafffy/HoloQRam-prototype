#pragma once

#include "hologram/compression/CompressionScheme.hpp"
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl-1.10/pcl/compression/octree_pointcloud_compression.h>
#include <memory>
#include <vector>

namespace hologram {

class OctreeCompression : public CompressionScheme {
public:
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    
    struct CompressionSettings {
        float resolution;          // Octree resolution
        bool doVoxelGridDownDownSampling;
        unsigned int iFrameRate;   // I-frame rate (number of P-frames before I-frame)
        bool doColorEncoding;      // Enable/disable color coding
        unsigned int colorBitResolution;
        
        CompressionSettings() 
            : resolution(0.01f)
            , doVoxelGridDownDownSampling(true)
            , iFrameRate(30)
            , doColorEncoding(true)
            , colorBitResolution(4) {}
    };
    
    OctreeCompression(const CompressionSettings& settings = CompressionSettings());
    
    // CompressionScheme interface implementation
    void compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) override;
    void decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) override;
    
    // Settings
    void updateSettings(const CompressionSettings& settings);
    const CompressionSettings& getSettings() const { return settings; }
    
    // Statistics
    float getCompressionRatio() const;
    size_t getCompressedSize() const;
    size_t getOriginalSize() const;
    
private:
    CompressionSettings settings;
    std::unique_ptr<pcl::io::OctreePointCloudCompression<PointT>> encoder;
    std::unique_ptr<pcl::io::OctreePointCloudCompression<PointT>> decoder;
    
    size_t lastOriginalSize;
    size_t lastCompressedSize;
    
    void initializeCompressor();
};

} // namespace hologram 