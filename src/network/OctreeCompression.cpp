#include "network/OctreeCompression.hpp"
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
    std::stringstream compressed;
    compressed.write(compressedData.data(), compressedData.size());
    decompressor.decodePointCloud(compressed, cloud);
} 