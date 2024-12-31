#include "hologram/compression/TemporalCompression.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

namespace hologram {

TemporalCompression::TemporalCompression(const Settings& settings)
    : settings(settings)
    , currentFrameIndex(0)
    , lastKeyframeIndex(0)
    , lastKeyframe(new CloudT)
    , stats{0.0f, 0, 0, 0.0f} {
    
    // Initialize change detector
    changeDetector = std::make_unique<OctreeT>(settings.octreeResolution);
}

TemporalCompression::CompressedFrame TemporalCompression::compressNextFrame(
    const CloudT::ConstPtr& cloud,
    const ViewportInfo& viewport) {
    
    CompressedFrame frame;
    frame.frameIndex = currentFrameIndex++;
    
    // Decide if we need a keyframe
    if (shouldGenerateKeyframe(cloud)) {
        frame = compressKeyframe(cloud, viewport);
        lastKeyframeIndex = frame.frameIndex;
        *lastKeyframe = *cloud; // Store keyframe for delta compression
    } else {
        frame = compressDelta(cloud, viewport);
    }
    
    // Update history
    updateHistory(frame);
    
    return frame;
}

bool TemporalCompression::shouldGenerateKeyframe(const CloudT::ConstPtr& cloud) const {
    // Always generate keyframe if we don't have one yet
    if (lastKeyframe->empty()) return true;
    
    // Check if we've reached the keyframe interval
    if (!settings.adaptiveKeyframes &&
        (currentFrameIndex - lastKeyframeIndex) >= settings.keyframeInterval) {
        return true;
    }
    
    // For adaptive keyframes, check the amount of change
    if (settings.adaptiveKeyframes) {
        std::vector<uint32_t> added, removed;
        const_cast<TemporalCompression*>(this)->detectChanges(cloud, added, removed);
        
        float changeRatio = static_cast<float>(added.size() + removed.size()) /
                           static_cast<float>(cloud->size());
        
        if (changeRatio > settings.changeThreshold) {
            return true;
        }
    }
    
    return false;
}

TemporalCompression::CompressedFrame TemporalCompression::compressKeyframe(
    const CloudT::ConstPtr& cloud,
    const ViewportInfo& viewport) {
    
    CompressedFrame frame;
    frame.type = CompressedFrame::FrameType::KEYFRAME;
    frame.frameIndex = currentFrameIndex;
    frame.referenceFrame = currentFrameIndex;
    
    // Compress the entire visible point cloud
    frame.data = visibilityCompressor.compressWithVisibility(cloud, viewport);
    
    // Update statistics
    stats.keyframeCount++;
    stats.totalCompressionRatio += visibilityCompressor.getCompressionRatio();
    stats.frameCount++;
    
    return frame;
}

TemporalCompression::CompressedFrame TemporalCompression::compressDelta(
    const CloudT::ConstPtr& cloud,
    const ViewportInfo& viewport) {
    
    CompressedFrame frame;
    frame.type = CompressedFrame::FrameType::DELTA;
    frame.frameIndex = currentFrameIndex;
    frame.referenceFrame = lastKeyframeIndex;
    
    // Detect changes between current frame and last keyframe
    detectChanges(cloud, frame.addedPoints, frame.removedPoints);
    
    // Extract changed points
    if (!frame.addedPoints.empty()) {
        pcl::PointIndices::Ptr addedIndices(new pcl::PointIndices);
        addedIndices->indices.assign(frame.addedPoints.begin(), frame.addedPoints.end());
        
        CloudT::Ptr addedCloud(new CloudT);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(addedIndices);
        extract.filter(*addedCloud);
        
        // Compress only the changed points
        frame.data = visibilityCompressor.compressWithVisibility(addedCloud, viewport);
    }
    
    // Update statistics
    float changeRatio = static_cast<float>(frame.addedPoints.size() + frame.removedPoints.size()) /
                       static_cast<float>(cloud->size());
    stats.lastChangeRatio = changeRatio;
    stats.totalCompressionRatio += visibilityCompressor.getCompressionRatio();
    stats.frameCount++;
    
    return frame;
}

void TemporalCompression::detectChanges(
    const CloudT::ConstPtr& cloud,
    std::vector<uint32_t>& added,
    std::vector<uint32_t>& removed) {
    
    // Reset change detector
    changeDetector->deleteTree();
    changeDetector->setInputCloud(lastKeyframe);
    changeDetector->addPointsFromInputCloud();
    
    // Switch to new cloud and get changes
    changeDetector->switchBuffers();
    changeDetector->setInputCloud(cloud);
    changeDetector->addPointsFromInputCloud();
    
    // Get point indices for changes
    std::vector<int> newPointIdxVector;
    changeDetector->getPointIndicesFromNewVoxels(newPointIdxVector);
    
    // Convert to uint32_t and store added points
    added.clear();
    added.reserve(newPointIdxVector.size());
    for (int idx : newPointIdxVector) {
        added.push_back(static_cast<uint32_t>(idx));
    }
    
    // Detect removed points by comparing with previous frame
    removed.clear();
    for (size_t i = 0; i < lastKeyframe->size(); ++i) {
        const auto& point = lastKeyframe->points[i];
        std::vector<int> pointIdx;
        std::vector<float> pointDist;
        
        if (changeDetector->radiusSearch(point, settings.octreeResolution * 0.5f, pointIdx, pointDist) == 0) {
            removed.push_back(static_cast<uint32_t>(i));
        }
    }
}

CloudT::Ptr TemporalCompression::decompress(const CompressedFrame& frame) {
    if (frame.type == CompressedFrame::FrameType::KEYFRAME) {
        // For keyframes, just decompress the entire cloud
        return visibilityCompressor.decompress(frame.data);
    } else {
        // For delta frames, apply changes to the reference keyframe
        CloudT::Ptr result(new CloudT(*lastKeyframe));
        
        // Remove points that were deleted
        if (!frame.removedPoints.empty()) {
            pcl::PointIndices::Ptr removedIndices(new pcl::PointIndices);
            removedIndices->indices.assign(frame.removedPoints.begin(), frame.removedPoints.end());
            
            CloudT::Ptr tempCloud(new CloudT);
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(result);
            extract.setIndices(removedIndices);
            extract.setNegative(true);  // Keep points that are NOT in indices
            extract.filter(*tempCloud);
            
            result = tempCloud;
        }
        
        // Add new points
        if (!frame.data.empty()) {
            CloudT::Ptr addedPoints = visibilityCompressor.decompress(frame.data);
            *result += *addedPoints;
        }
        
        return result;
    }
}

void TemporalCompression::updateHistory(const CompressedFrame& frame) {
    frameHistory.push_back(frame);
    while (frameHistory.size() > settings.historyLength) {
        frameHistory.pop_front();
    }
}

CloudT::Ptr TemporalCompression::getLastKeyframe() const {
    return CloudT::Ptr(new CloudT(*lastKeyframe));
}

void TemporalCompression::updateSettings(const Settings& newSettings) {
    settings = newSettings;
    changeDetector = std::make_unique<OctreeT>(settings.octreeResolution);
}

float TemporalCompression::getAverageCompressionRatio() const {
    if (stats.frameCount == 0) return 0.0f;
    return stats.totalCompressionRatio / static_cast<float>(stats.frameCount);
}

float TemporalCompression::getChangeRatio() const {
    return stats.lastChangeRatio;
}

size_t TemporalCompression::getKeyframeCount() const {
    return stats.keyframeCount;
}

} // namespace hologram 