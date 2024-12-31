#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include "hologram/compression/VisibilityAwareCompression.hpp"
#include <deque>

namespace hologram {

class TemporalCompression {
public:
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    
    struct Settings {
        float octreeResolution;     // Resolution for change detection
        size_t keyframeInterval;    // Number of frames between keyframes
        float changeThreshold;      // Minimum change ratio to trigger update
        size_t historyLength;       // Number of frames to keep in history
        bool adaptiveKeyframes;     // Enable adaptive keyframe selection
        
        Settings()
            : octreeResolution(0.01f)
            , keyframeInterval(30)
            , changeThreshold(0.1f)
            , historyLength(5)
            , adaptiveKeyframes(true) {}
    };
    
    struct CompressedFrame {
        enum class FrameType {
            KEYFRAME,
            DELTA
        };
        
        FrameType type;
        uint32_t frameIndex;
        uint32_t referenceFrame;    // For delta frames
        std::vector<char> data;
        std::vector<uint32_t> addedPoints;
        std::vector<uint32_t> removedPoints;
    };
    
    TemporalCompression(const Settings& settings = Settings());
    
    // Main compression interface
    CompressedFrame compressNextFrame(
        const CloudT::ConstPtr& cloud,
        const ViewportInfo& viewport);
        
    CloudT::Ptr decompress(const CompressedFrame& frame);
    CloudT::Ptr getLastKeyframe() const;
    
    // Settings
    void updateSettings(const Settings& settings);
    const Settings& getSettings() const { return settings; }
    
    // Statistics
    float getAverageCompressionRatio() const;
    float getChangeRatio() const;
    size_t getKeyframeCount() const;
    
private:
    Settings settings;
    VisibilityAwareCompression visibilityCompressor;
    
    // Frame management
    uint32_t currentFrameIndex;
    uint32_t lastKeyframeIndex;
    CloudT::Ptr lastKeyframe;
    std::deque<CompressedFrame> frameHistory;
    
    // Change detection
    using OctreeT = pcl::octree::OctreePointCloudChangeDetector<PointT>;
    std::unique_ptr<OctreeT> changeDetector;
    
    // Temporal processing
    bool shouldGenerateKeyframe(const CloudT::ConstPtr& cloud) const;
    CompressedFrame compressKeyframe(const CloudT::ConstPtr& cloud, const ViewportInfo& viewport);
    CompressedFrame compressDelta(const CloudT::ConstPtr& cloud, const ViewportInfo& viewport);
    void detectChanges(const CloudT::ConstPtr& cloud, std::vector<uint32_t>& added, std::vector<uint32_t>& removed);
    void updateHistory(const CompressedFrame& frame);
    
    // Statistics
    struct Stats {
        float totalCompressionRatio;
        size_t frameCount;
        size_t keyframeCount;
        float lastChangeRatio;
    } stats;
};

} // namespace hologram 