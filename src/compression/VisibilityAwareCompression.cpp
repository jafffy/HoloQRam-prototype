#include "hologram/compression/VisibilityAwareCompression.hpp"
#include <glm/gtc/matrix_access.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <unordered_map>

namespace hologram {

VisibilityAwareCompression::VisibilityAwareCompression(const Settings& settings)
    : settings(settings)
    , originalPoints(0)
    , visiblePoints(0)
    , lastCompressedSize(0) {
    
    // Initialize octree compressor with base settings
    OctreeCompression::CompressionSettings octreeSettings;
    octreeSettings.resolution = settings.octreeResolution;
    octreeCompressor.updateSettings(octreeSettings);
}

std::vector<char> VisibilityAwareCompression::compressWithVisibility(
    const CloudT::ConstPtr& cloud,
    const ViewportInfo& viewport) {
    
    originalPoints = cloud->size();
    
    // Process visibility and get visible points
    CloudT::Ptr visibleCloud = processVisibility(cloud, viewport);
    visiblePoints = visibleCloud->size();
    
    // Compress the visible points
    return octreeCompressor.compress(visibleCloud);
}

CloudT::Ptr VisibilityAwareCompression::processVisibility(
    const CloudT::ConstPtr& cloud,
    const ViewportInfo& viewport) {
    
    // Create octree for spatial partitioning
    OctreeT octree(settings.octreeResolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    
    // Compute visibility for each cell
    computeCellVisibility(octree, viewport);
    
    // Create indices for visible points
    pcl::PointIndices::Ptr visibleIndices(new pcl::PointIndices);
    
    // Iterate through octree leaves
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        std::vector<int> pointIdxVec;
        octree.getPointIndicesFromOctreeKey(it.getCurrentOctreeKey(), pointIdxVec);
        
        // Get cell center
        Eigen::Vector3f cellCenter;
        octree.getOctreeCenterFromOctreeKey(it.getCurrentOctreeKey(), cellCenter);
        
        // Convert to PCL point for visibility test
        PointT centerPoint;
        centerPoint.x = cellCenter[0];
        centerPoint.y = cellCenter[1];
        centerPoint.z = cellCenter[2];
        
        if (isPointVisible(centerPoint, viewport)) {
            float importance = computeDistanceImportance(centerPoint, viewport);
            float occlusion = computeOcclusion(octree, {pointIdxVec, importance, true, 0, 0}, viewport);
            
            // If cell is visible and not fully occluded
            if (occlusion < settings.occlusionThreshold) {
                // Add all points in the cell
                visibleIndices->indices.insert(
                    visibleIndices->indices.end(),
                    pointIdxVec.begin(),
                    pointIdxVec.end()
                );
            }
        }
    }
    
    // Extract visible points
    CloudT::Ptr visibleCloud(new CloudT);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(visibleIndices);
    extract.filter(*visibleCloud);
    
    return visibleCloud;
}

void VisibilityAwareCompression::computeCellVisibility(
    OctreeT& octree,
    const ViewportInfo& viewport) {
    
    glm::mat4 viewProj = viewport.getProjectionMatrix() * viewport.getViewMatrix();
    
    // Transform all points to clip space for visibility test
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        Eigen::Vector3f center;
        octree.getOctreeCenterFromOctreeKey(it.getCurrentOctreeKey(), center);
        
        // Convert to homogeneous coordinates
        glm::vec4 pos(center[0], center[1], center[2], 1.0f);
        
        // Transform to clip space
        glm::vec4 clipPos = viewProj * pos;
        
        // Perspective divide
        if (clipPos.w > 0) {
            clipPos /= clipPos.w;
            
            // Check if point is in view frustum
            bool visible = clipPos.x >= -1.0f && clipPos.x <= 1.0f &&
                         clipPos.y >= -1.0f && clipPos.y <= 1.0f &&
                         clipPos.z >= 0.0f && clipPos.z <= 1.0f;
            
            if (visible) {
                std::vector<int> pointIdxVec;
                octree.getPointIndicesFromOctreeKey(it.getCurrentOctreeKey(), pointIdxVec);
                
                // Store visibility information
                OctreeCell cell;
                cell.pointIndices.assign(pointIdxVec.begin(), pointIdxVec.end());
                cell.isVisible = true;
                cell.distance = glm::length(glm::vec3(pos) - viewport.position);
                cell.importance = computeDistanceImportance(
                    PointT{center[0], center[1], center[2]}, viewport);
            }
        }
    }
}

bool VisibilityAwareCompression::isPointVisible(
    const PointT& point,
    const ViewportInfo& viewport) const {
    
    glm::vec3 pointPos(point.x, point.y, point.z);
    glm::vec3 toPoint = glm::normalize(pointPos - viewport.position);
    
    // Check if point is within FOV
    float angle = glm::acos(glm::dot(toPoint, viewport.direction));
    if (angle > viewport.fov * 0.5f) {
        return false;
    }
    
    // Check distance
    float distance = glm::length(pointPos - viewport.position);
    if (distance < viewport.nearPlane || distance > viewport.farPlane) {
        return false;
    }
    
    return true;
}

float VisibilityAwareCompression::computeOcclusion(
    const OctreeT& octree,
    const OctreeCell& cell,
    const ViewportInfo& viewport) const {
    
    // Simple occlusion estimation based on density of points between cell and camera
    float occlusion = 0.0f;
    
    // Get points along ray from camera to cell center
    Eigen::Vector3f rayStart = Eigen::Vector3f(
        viewport.position.x,
        viewport.position.y,
        viewport.position.z);
        
    // Sample points along ray
    const int numSamples = 10;
    float totalDensity = 0.0f;
    
    for (int i = 1; i < numSamples; ++i) {
        float t = static_cast<float>(i) / numSamples;
        Eigen::Vector3f sample = rayStart + t * (Eigen::Vector3f(cell.distance, 0, 0) - rayStart);
        
        std::vector<int> pointIdxVec;
        octree.radiusSearch(sample, settings.octreeResolution * 2.0f, pointIdxVec);
        
        totalDensity += static_cast<float>(pointIdxVec.size());
    }
    
    occlusion = totalDensity / (numSamples * 10.0f); // Normalize by expected max density
    return std::min(occlusion, 1.0f);
}

float VisibilityAwareCompression::computeDistanceImportance(
    const PointT& point,
    const ViewportInfo& viewport) const {
    
    glm::vec3 pointPos(point.x, point.y, point.z);
    float distance = glm::length(pointPos - viewport.position);
    
    // Inverse square falloff with distance
    float importance = 1.0f / (1.0f + distance * distance * settings.distanceWeight);
    return importance;
}

CloudT::Ptr VisibilityAwareCompression::decompress(const std::vector<char>& data) {
    return octreeCompressor.decompress(data);
}

void VisibilityAwareCompression::updateSettings(const Settings& newSettings) {
    settings = newSettings;
    
    // Update octree compressor settings
    OctreeCompression::CompressionSettings octreeSettings;
    octreeSettings.resolution = settings.octreeResolution;
    octreeCompressor.updateSettings(octreeSettings);
}

float VisibilityAwareCompression::getCompressionRatio() const {
    return octreeCompressor.getCompressionRatio();
}

size_t VisibilityAwareCompression::getCompressedSize() const {
    return octreeCompressor.getCompressedSize();
}

size_t VisibilityAwareCompression::getOriginalSize() const {
    return octreeCompressor.getOriginalSize();
}

float VisibilityAwareCompression::getVisiblePointsRatio() const {
    if (originalPoints == 0) return 0.0f;
    return static_cast<float>(visiblePoints) / static_cast<float>(originalPoints);
}

} // namespace hologram 