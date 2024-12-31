#include "hologram/server/CellManager.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

namespace hologram {

CellManager::CellManager(float cellSize) : cellSize(cellSize) {}

void CellManager::segmentPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    cells.clear();
    
    // Group points into cells based on their position
    for (const auto& point : *cloud) {
        glm::vec3 pos(point.x, point.y, point.z);
        CellKey key = getCellKey(pos);
        
        auto& cell = cells[key];
        cell.center = glm::vec3(
            (key.x + 0.5f) * cellSize,
            (key.y + 0.5f) * cellSize,
            (key.z + 0.5f) * cellSize
        );
        cell.size = cellSize;
        cell.points->push_back(point);
    }
}

void CellManager::updateCellVisibility(const ViewportData& viewport) {
    for (auto& [key, cell] : cells) {
        cell.isVisible = isCellVisible(cell, viewport);
        if (cell.isVisible) {
            updateCellImportance(cell, viewport);
        }
    }
}

std::vector<const Cell*> CellManager::getVisibleCells(const ViewportData& viewport) const {
    std::vector<const Cell*> visibleCells;
    visibleCells.reserve(cells.size());
    
    for (const auto& [key, cell] : cells) {
        if (cell.isVisible) {
            visibleCells.push_back(&cell);
        }
    }
    
    // Sort by importance (highest first)
    std::sort(visibleCells.begin(), visibleCells.end(),
              [](const Cell* a, const Cell* b) {
                  return a->importance > b->importance;
              });
    
    return visibleCells;
}

const Cell* CellManager::getCell(const glm::vec3& center) const {
    auto it = cells.find(getCellKey(center));
    return it != cells.end() ? &it->second : nullptr;
}

void CellManager::removeCell(const glm::vec3& center) {
    cells.erase(getCellKey(center));
}

void CellManager::setCellSize(float size) {
    if (size != cellSize) {
        cellSize = size;
        // Re-segment point cloud if we have any cells
        if (!cells.empty()) {
            auto combinedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (const auto& [key, cell] : cells) {
                *combinedCloud += *cell.points;
            }
            segmentPointCloud(combinedCloud);
        }
    }
}

bool CellManager::isCellVisible(const Cell& cell, const ViewportData& viewport) const {
    // Simple frustum culling
    glm::mat4 view = glm::lookAt(viewport.position,
                                viewport.position + viewport.direction,
                                glm::vec3(0, 1, 0));
    glm::mat4 proj = glm::perspective(viewport.fov,
                                    viewport.aspectRatio,
                                    viewport.nearPlane,
                                    viewport.farPlane);
    glm::mat4 viewProj = proj * view;
    
    // Check if cell's bounding box corners are in view frustum
    std::vector<glm::vec4> corners = {
        glm::vec4(cell.center.x - cell.size/2, cell.center.y - cell.size/2, cell.center.z - cell.size/2, 1.0f),
        glm::vec4(cell.center.x + cell.size/2, cell.center.y - cell.size/2, cell.center.z - cell.size/2, 1.0f),
        glm::vec4(cell.center.x - cell.size/2, cell.center.y + cell.size/2, cell.center.z - cell.size/2, 1.0f),
        glm::vec4(cell.center.x + cell.size/2, cell.center.y + cell.size/2, cell.center.z - cell.size/2, 1.0f),
        glm::vec4(cell.center.x - cell.size/2, cell.center.y - cell.size/2, cell.center.z + cell.size/2, 1.0f),
        glm::vec4(cell.center.x + cell.size/2, cell.center.y - cell.size/2, cell.center.z + cell.size/2, 1.0f),
        glm::vec4(cell.center.x - cell.size/2, cell.center.y + cell.size/2, cell.center.z + cell.size/2, 1.0f),
        glm::vec4(cell.center.x + cell.size/2, cell.center.y + cell.size/2, cell.center.z + cell.size/2, 1.0f)
    };
    
    bool anyCornerVisible = false;
    for (const auto& corner : corners) {
        glm::vec4 clipSpace = viewProj * corner;
        if (clipSpace.w <= 0) continue;
        
        glm::vec3 ndc(clipSpace.x / clipSpace.w,
                     clipSpace.y / clipSpace.w,
                     clipSpace.z / clipSpace.w);
        
        if (ndc.x >= -1 && ndc.x <= 1 &&
            ndc.y >= -1 && ndc.y <= 1 &&
            ndc.z >= -1 && ndc.z <= 1) {
            anyCornerVisible = true;
            break;
        }
    }
    
    return anyCornerVisible;
}

float CellManager::computeCellImportance(const Cell& cell, const ViewportData& viewport) const {
    // Compute importance based on:
    // 1. Distance from viewport
    // 2. Angle from view direction
    // 3. Number of points in cell
    
    float distance = glm::length(cell.center - viewport.position);
    float angle = glm::dot(glm::normalize(cell.center - viewport.position),
                          viewport.direction);
    
    // Normalize factors
    float distanceFactor = 1.0f / (1.0f + distance * 0.01f); // Decrease importance with distance
    float angleFactor = (angle + 1.0f) * 0.5f; // Map [-1,1] to [0,1]
    float densityFactor = std::min(1.0f, cell.points->size() / 1000.0f); // Cap at 1000 points
    
    // Weighted combination
    return (0.4f * distanceFactor +
            0.4f * angleFactor +
            0.2f * densityFactor);
}

void CellManager::updateCellImportance(Cell& cell, const ViewportData& viewport) {
    cell.importance = computeCellImportance(cell, viewport);
}

} // namespace hologram 