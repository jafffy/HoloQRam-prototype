#include "hologram/compression/ViVoCompression.hpp"
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

namespace hologram {

ViVoCompression::ViVoCompression(const Settings& settings) : settings(settings) {}

void ViVoCompression::compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) {
    // Create a new point cloud for visible points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visibleCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Process visibility and filter points
    for (const auto& point : cloud->points) {
        if (isPointVisible(point)) {
            float importance = computeDistanceImportance(point);
            float occlusion = computeOcclusion(point);
            
            if (importance > settings.visibilityThreshold && occlusion < settings.occlusionThreshold) {
                visibleCloud->points.push_back(point);
            }
        }
    }
    
    // Update cloud metadata
    visibleCloud->width = visibleCloud->points.size();
    visibleCloud->height = 1;
    visibleCloud->is_dense = false;
    
    // Compress using octree-based compression
    float resolution = settings.octreeResolution;
    if (settings.adaptiveResolution) {
        // Adjust resolution based on point cloud size
        resolution *= std::max(1.0f, std::log10(static_cast<float>(visibleCloud->points.size())));
    }
    
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB> octree(resolution);
    octree.setInputCloud(visibleCloud);
    octree.addPointsFromInputCloud();
    
    // Serialize octree to binary data
    octree.serializeTree(compressedData);
}

void ViVoCompression::decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if (!cloud) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    
    if (compressedData.empty()) {
        cloud->clear();
        return;
    }
    
    // Create octree with same resolution
    float resolution = settings.octreeResolution;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB> octree(resolution);
    
    // Create a non-const copy of the compressed data for deserialization
    std::vector<char> mutableData = compressedData;
    
    // Deserialize tree
    octree.deserializeTree(mutableData);
    
    // Get points from octree
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> points;
    octree.getVoxelCentroids(points);
    
    // Convert to point cloud
    cloud->points.assign(points.begin(), points.end());
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}

void ViVoCompression::updateSettings(const Settings& newSettings) {
    settings = newSettings;
}

bool ViVoCompression::isPointVisible(const pcl::PointXYZRGB& point) const {
    // Simple visibility check based on point position
    // In a real implementation, this would use view frustum culling and depth testing
    return true;
}

float ViVoCompression::computeOcclusion(const pcl::PointXYZRGB& point) const {
    // Simple occlusion estimation
    // In a real implementation, this would use ray casting or depth buffer comparison
    return 0.0f;
}

float ViVoCompression::computeDistanceImportance(const pcl::PointXYZRGB& point) const {
    // Simple distance-based importance
    // In a real implementation, this would consider camera position and view direction
    return 1.0f;
}

} // namespace hologram 