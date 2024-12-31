#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>
#include <string>

namespace hologram {

class CompressionScheme {
public:
    virtual ~CompressionScheme() = default;
    
    virtual void compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) = 0;
    virtual void decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) = 0;
    
    static std::unique_ptr<CompressionScheme> create(const std::string& scheme);
};

} // namespace hologram 