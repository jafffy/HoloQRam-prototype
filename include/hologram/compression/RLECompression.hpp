#pragma once

#include "hologram/compression/CompressionScheme.hpp"

namespace hologram {

class RLECompression : public CompressionScheme {
public:
    RLECompression() = default;
    
    void compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) override;
    void decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) override;
    
private:
    struct Point {
        float x, y, z;
        uint8_t r, g, b;
    };
    
    void encodeRLE(const std::vector<Point>& points, std::vector<char>& compressed);
    void decodeRLE(const std::vector<char>& compressed, std::vector<Point>& points);
};

} // namespace hologram 