#pragma once

// Standard includes first
#include <vector>
#include <memory>

// Boost includes (before PCL to avoid namespace issues)
#include <boost/concept_check.hpp>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>

// GLM includes
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Project includes
#include "hologram/compression/CompressionScheme.hpp"

namespace hologram {

class ViVoCompression : public CompressionScheme {
public:
    struct Settings {
        float octreeResolution;    // Base octree resolution
        float visibilityThreshold; // Threshold for visibility test (0-1)
        float distanceWeight;      // Weight for distance-based importance (0-1)
        float occlusionThreshold;  // Threshold for occlusion test (0-1)
        bool adaptiveResolution;   // Enable adaptive resolution based on distance
        
        Settings()
            : octreeResolution(0.01f)
            , visibilityThreshold(0.1f)
            , distanceWeight(0.5f)
            , occlusionThreshold(0.5f)
            , adaptiveResolution(true) {}
    };
    
    explicit ViVoCompression(const Settings& settings = Settings());
    
    void compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) override;
    void decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) override;
    
    // Settings
    void updateSettings(const Settings& settings);
    const Settings& getSettings() const { return settings; }
    
private:
    Settings settings;
    
    // Helper functions
    void processVisibility(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    bool isPointVisible(const pcl::PointXYZRGB& point) const;
    float computeOcclusion(const pcl::PointXYZRGB& point) const;
    float computeDistanceImportance(const pcl::PointXYZRGB& point) const;
};

} // namespace hologram 