#pragma once

#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/octree/octree_pointcloud.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "hologram/compression/OctreeCompression.hpp"
#include "hologram/utils/GlmHash.hpp"

namespace hologram {

struct ViewportInfo {
    glm::vec3 position;      // Camera position
    glm::vec3 direction;     // View direction
    float fov;              // Field of view in radians
    float aspectRatio;      // Viewport aspect ratio
    float nearPlane;        // Near plane distance
    float farPlane;         // Far plane distance
    
    glm::mat4 getViewMatrix() const {
        return glm::lookAt(position, position + direction, glm::vec3(0, 1, 0));
    }
    
    glm::mat4 getProjectionMatrix() const {
        return glm::perspective(fov, aspectRatio, nearPlane, farPlane);
    }
};

class VisibilityAwareCompression {
public:
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    
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
    
    VisibilityAwareCompression(const Settings& settings = Settings());
    
    // Main compression interface
    std::vector<char> compressWithVisibility(
        const CloudT::ConstPtr& cloud,
        const ViewportInfo& viewport);
        
    CloudT::Ptr decompress(const std::vector<char>& data);
    
    // Settings
    void updateSettings(const Settings& settings);
    const Settings& getSettings() const { return settings; }
    
    // Statistics
    float getCompressionRatio() const;
    size_t getCompressedSize() const;
    size_t getOriginalSize() const;
    float getVisiblePointsRatio() const;
    
private:
    Settings settings;
    OctreeCompression octreeCompressor;
    
    // Visibility computation
    struct OctreeCell {
        std::vector<size_t> pointIndices;
        float importance;
        bool isVisible;
        float distance;
        float occlusion;
    };
    
    using OctreeT = pcl::octree::OctreePointCloud<PointT>;
    
    // Visibility processing
    CloudT::Ptr processVisibility(const CloudT::ConstPtr& cloud, const ViewportInfo& viewport);
    void computeCellVisibility(OctreeT& octree, const ViewportInfo& viewport);
    bool isPointVisible(const PointT& point, const ViewportInfo& viewport) const;
    float computeOcclusion(const OctreeT& octree, const OctreeCell& cell, const ViewportInfo& viewport) const;
    float computeDistanceImportance(const PointT& point, const ViewportInfo& viewport) const;
    
    // Statistics
    size_t originalPoints;
    size_t visiblePoints;
    size_t lastCompressedSize;
};

class CellReconstructor {
public:
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    
    struct Cell {
        CloudT::Ptr points;
        glm::vec3 center;
        float size;
        float importance;
        bool isVisible;
        
        Cell() : points(new CloudT()), size(0.0f), importance(0.0f), isVisible(true) {}
    };
    
    struct Settings {
        float cellSize;           // Size of each cell in meters
        size_t maxCellPoints;     // Maximum points per cell for LOD
        float visibilityThreshold;// Minimum visibility value to render
        bool enableLOD;          // Enable Level of Detail based on distance
        
        Settings()
            : cellSize(1.0f)
            , maxCellPoints(10000)
            , visibilityThreshold(0.1f)
            , enableLOD(true) {}
    };
    
    explicit CellReconstructor(const Settings& settings = Settings());
    
    // Cell management
    void addCell(const std::vector<char>& compressedData, const glm::vec3& cellCenter);
    void updateCellVisibility(const ViewportInfo& viewport);
    void removeCell(const glm::vec3& cellCenter);
    void clear();
    
    // Reconstruction
    CloudT::Ptr reconstructPointCloud(const ViewportInfo& viewport);
    
    // Settings
    void updateSettings(const Settings& settings);
    const Settings& getSettings() const { return settings; }
    
private:
    Settings settings;
    std::unordered_map<glm::vec3, Cell> cells;  // Using cell center as key
    
    // Helper functions
    float computeCellImportance(const Cell& cell, const ViewportInfo& viewport) const;
    size_t computeCellLOD(const Cell& cell, const ViewportInfo& viewport) const;
    bool isCellVisible(const Cell& cell, const ViewportInfo& viewport) const;
    CloudT::Ptr samplePoints(const CloudT::Ptr& input, size_t targetCount) const;
};

} // namespace hologram 