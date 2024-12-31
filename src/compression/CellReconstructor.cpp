#include "hologram/compression/VisibilityAwareCompression.hpp"
#include "hologram/compression/CompressionScheme.hpp"
#include <pcl/filters/random_sample.h>
#include <glm/gtx/norm.hpp>

namespace hologram {

CellReconstructor::CellReconstructor(const Settings& settings)
    : settings(settings) {
}

void CellReconstructor::addCell(const std::vector<char>& compressedData, const glm::vec3& cellCenter) {
    Cell cell;
    cell.center = cellCenter;
    cell.size = settings.cellSize;
    
    try {
        // Decompress the cell data using the ViVo compression scheme
        auto compressor = CompressionScheme::create("vivo");
        compressor->decompress(compressedData, cell.points);
        
        // Store the cell
        cells[cellCenter] = std::move(cell);
    } catch (const std::exception& e) {
        std::cerr << "Error decompressing cell data: " << e.what() << std::endl;
    }
}

void CellReconstructor::updateCellVisibility(const ViewportInfo& viewport) {
    for (auto& [center, cell] : cells) {
        cell.isVisible = isCellVisible(cell, viewport);
        if (cell.isVisible) {
            cell.importance = computeCellImportance(cell, viewport);
        } else {
            cell.importance = 0.0f;
        }
    }
}

void CellReconstructor::removeCell(const glm::vec3& cellCenter) {
    cells.erase(cellCenter);
}

void CellReconstructor::clear() {
    cells.clear();
}

CloudT::Ptr CellReconstructor::reconstructPointCloud(const ViewportInfo& viewport) {
    CloudT::Ptr result(new CloudT);
    
    // Update visibility and importance for all cells
    updateCellVisibility(viewport);
    
    // Combine visible cells with LOD
    for (const auto& [center, cell] : cells) {
        if (!cell.isVisible || cell.importance < settings.visibilityThreshold) {
            continue;
        }
        
        CloudT::Ptr cellCloud = cell.points;
        if (settings.enableLOD) {
            size_t targetPoints = computeCellLOD(cell, viewport);
            if (cellCloud->size() > targetPoints) {
                cellCloud = samplePoints(cell.points, targetPoints);
            }
        }
        
        *result += *cellCloud;
    }
    
    return result;
}

void CellReconstructor::updateSettings(const Settings& newSettings) {
    settings = newSettings;
}

float CellReconstructor::computeCellImportance(const Cell& cell, const ViewportInfo& viewport) const {
    // Calculate distance-based importance
    float distance = glm::length(cell.center - glm::vec3(viewport.position.x, viewport.position.y, viewport.position.z));
    float maxDistance = viewport.farPlane;
    float distanceImportance = 1.0f - std::min(distance / maxDistance, 1.0f);
    
    // Calculate angle-based importance
    glm::vec3 toCell = glm::normalize(cell.center - glm::vec3(viewport.position.x, viewport.position.y, viewport.position.z));
    glm::vec3 viewDir = glm::vec3(viewport.direction.x, viewport.direction.y, viewport.direction.z);
    float angle = glm::acos(glm::dot(toCell, viewDir));
    float angleImportance = 1.0f - std::min(angle / (viewport.fov * 0.5f), 1.0f);
    
    // Combine importance factors
    return distanceImportance * angleImportance;
}

size_t CellReconstructor::computeCellLOD(const Cell& cell, const ViewportInfo& viewport) const {
    float importance = cell.importance;
    return static_cast<size_t>(settings.maxCellPoints * importance);
}

bool CellReconstructor::isCellVisible(const Cell& cell, const ViewportInfo& viewport) const {
    // Simple frustum culling
    glm::vec4 cellPos = glm::vec4(cell.center, 1.0f);
    glm::mat4 viewProj = viewport.getProjectionMatrix() * viewport.getViewMatrix();
    glm::vec4 clipPos = viewProj * cellPos;
    
    // Check if cell center is within the view frustum
    float w = clipPos.w;
    return clipPos.x >= -w && clipPos.x <= w &&
           clipPos.y >= -w && clipPos.y <= w &&
           clipPos.z >= -w && clipPos.z <= w;
}

CloudT::Ptr CellReconstructor::samplePoints(const CloudT::Ptr& input, size_t targetCount) const {
    if (input->empty() || targetCount >= input->size()) {
        return input;
    }
    
    CloudT::Ptr output(new CloudT);
    pcl::RandomSample<PointT> sampler;
    sampler.setInputCloud(input);
    sampler.setSample(static_cast<unsigned int>(targetCount));
    sampler.filter(*output);
    
    return output;
}

} // namespace hologram 