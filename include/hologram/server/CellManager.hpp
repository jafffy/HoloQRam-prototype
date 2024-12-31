#pragma once

// PCL includes
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/point_cloud.h>

// GLM includes
#include <glm/glm.hpp>

// Standard library includes
#include <vector>
#include <memory>
#include <unordered_map>
#include <tuple>

namespace hologram {

struct ViewportData {
    glm::vec3 position;
    glm::vec3 direction;
    float fov;
    float aspectRatio;
    float nearPlane;
    float farPlane;
};

struct Cell {
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    
    CloudT::Ptr points;
    glm::vec3 center;
    float size;
    float importance;
    bool isVisible;
    
    Cell() : points(new CloudT()), size(0.0f), importance(0.0f), isVisible(true) {}
};

// Custom key type for cells
struct CellKey {
    int x, y, z;
    
    CellKey(const glm::vec3& pos, float cellSize)
        : x(static_cast<int>(std::floor(pos.x / cellSize)))
        , y(static_cast<int>(std::floor(pos.y / cellSize)))
        , z(static_cast<int>(std::floor(pos.z / cellSize))) {}
        
    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

} // namespace hologram

// Hash function for CellKey
namespace std {
    template<>
    struct hash<hologram::CellKey> {
        size_t operator()(const hologram::CellKey& k) const noexcept {
            return static_cast<size_t>(
                k.x * 73856093 ^
                k.y * 19349663 ^
                k.z * 83492791
            );
        }
    };
}

namespace hologram {

class CellManager {
public:
    explicit CellManager(float cellSize);
    
    // Point cloud management
    void segmentPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void updateCellVisibility(const ViewportData& viewport);
    std::vector<const Cell*> getVisibleCells(const ViewportData& viewport) const;
    
    // Cell access
    const Cell* getCell(const glm::vec3& center) const;
    void removeCell(const glm::vec3& center);
    
    // Settings
    float getCellSize() const { return cellSize; }
    void setCellSize(float size);
    
private:
    float cellSize;
    std::unordered_map<CellKey, Cell> cells;
    
    // Helper functions
    bool isCellVisible(const Cell& cell, const ViewportData& viewport) const;
    float computeCellImportance(const Cell& cell, const ViewportData& viewport) const;
    void updateCellImportance(Cell& cell, const ViewportData& viewport);
    CellKey getCellKey(const glm::vec3& pos) const { return CellKey(pos, cellSize); }
};

} // namespace hologram 