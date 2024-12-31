#include "hologram/compression/OctreeCompression.hpp"
#include <sstream>

namespace hologram {

OctreeCompression::OctreeCompression(const CompressionSettings& settings) : settings(settings) {
    initializeCompressor();
}

void OctreeCompression::compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) {
    lastOriginalSize = cloud->size() * sizeof(pcl::PointXYZRGB);
    
    std::stringstream compressedStream;
    encoder->encodePointCloud(cloud, compressedStream);
    
    // Convert stringstream to vector<char>
    compressedData = std::vector<char>(
        std::istreambuf_iterator<char>(compressedStream),
        std::istreambuf_iterator<char>()
    );
    
    lastCompressedSize = compressedData.size();
}

void OctreeCompression::decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    // Convert vector<char> to stringstream
    std::stringstream compressedStream;
    compressedStream.write(compressedData.data(), compressedData.size());
    
    // Create a new cloud if needed
    if (!cloud) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    
    decoder->decodePointCloud(compressedStream, cloud);
}

void OctreeCompression::updateSettings(const CompressionSettings& newSettings) {
    settings = newSettings;
    initializeCompressor();
}

float OctreeCompression::getCompressionRatio() const {
    if (lastOriginalSize == 0) return 0.0f;
    return static_cast<float>(lastCompressedSize) / static_cast<float>(lastOriginalSize);
}

size_t OctreeCompression::getCompressedSize() const {
    return lastCompressedSize;
}

size_t OctreeCompression::getOriginalSize() const {
    return lastOriginalSize;
}

void OctreeCompression::initializeCompressor() {
    encoder.reset(new pcl::io::OctreePointCloudCompression<PointT>(
        pcl::io::MANUAL_CONFIGURATION,
        false, // showStatistics
        static_cast<double>(settings.resolution),
        static_cast<double>(settings.resolution),
        settings.doVoxelGridDownDownSampling,
        settings.iFrameRate,
        settings.doColorEncoding,
        static_cast<unsigned char>(settings.colorBitResolution)
    ));
    
    decoder.reset(new pcl::io::OctreePointCloudCompression<PointT>());
}

} // namespace hologram 