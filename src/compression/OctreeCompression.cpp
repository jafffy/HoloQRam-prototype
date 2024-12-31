#include "hologram/compression/OctreeCompression.hpp"
#include <sstream>

OctreeCompression::OctreeCompression()
    : compressor(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR, false)
    , decompressor(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR, false)
{
}

void OctreeCompression::compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) {
    std::stringstream compressed;
    compressor.encodePointCloud(cloud, compressed);
    
    // Convert stringstream to vector<char>
    compressed.seekg(0, std::ios::end);
    size_t size = compressed.tellg();
    compressed.seekg(0);
    
    compressedData.resize(size);
    compressed.read(compressedData.data(), size);
}

void OctreeCompression::decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    // Clear existing cloud and reserve approximate space
    cloud->clear();
    cloud->reserve(100000);  // Reserve space for 100k points - adjust based on your typical point cloud size
    
    // Create stringstream from compressed data
    std::stringstream compressed;
    compressed.write(compressedData.data(), compressedData.size());
    
    try {
        decompressor.decodePointCloud(compressed, cloud);
    } catch (const std::bad_alloc& e) {
        // If allocation fails, try with a smaller reserve
        cloud->clear();
        cloud->reserve(50000);  // Try with 50k points
        compressed.seekg(0);  // Reset stream position
        decompressor.decodePointCloud(compressed, cloud);
    }
    
    // Shrink to fit actual size
    cloud->points.shrink_to_fit();
} 