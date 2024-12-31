#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <vector>
#include <condition_variable>

class DecompressionManager {
public:
    static constexpr size_t MAX_COMPRESSED_FRAMES = 30;
    static constexpr size_t MAX_DECOMPRESSED_FRAMES = 10;

    DecompressionManager();
    ~DecompressionManager();
    
    void start();
    void stop();
    
    void addCompressedFrame(std::vector<char>&& compressedData);
    bool getNextDecompressedFrame(std::vector<float>& vertices);

private:
    void decompressFrames();
    
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> decompressor;
    std::thread decompressionThread;
    
    std::mutex compressedFramesMutex;
    std::mutex decompressedFramesMutex;
    std::condition_variable compressedFramesCV;
    std::condition_variable decompressedFramesCV;
    
    std::deque<std::vector<char>> compressedFrames;
    std::deque<std::vector<float>> decompressedFrames;
    
    std::atomic<bool> shouldStop;
}; 