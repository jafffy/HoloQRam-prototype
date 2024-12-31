#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <vector>
#include <condition_variable>
#include "network/CompressionScheme.hpp"

class DecompressionManager {
public:
    static constexpr size_t MAX_COMPRESSED_FRAMES = 30;
    static constexpr size_t MAX_DECOMPRESSED_FRAMES = 10;

    DecompressionManager(const std::string& compressionScheme = "octree");
    ~DecompressionManager();
    
    void start();
    void stop();
    
    void addCompressedFrame(std::vector<char>&& compressedData);
    bool getNextDecompressedFrame(std::vector<float>& vertices);
    
    // Queue size getters for performance metrics
    size_t getCompressedQueueSize() const {
        std::lock_guard<std::mutex> lock(compressedFramesMutex);
        return compressedFrames.size();
    }
    
    size_t getDecompressedQueueSize() const {
        std::lock_guard<std::mutex> lock(decompressedFramesMutex);
        return decompressedFrames.size();
    }

private:
    void decompressFrames();
    
    std::unique_ptr<CompressionScheme> decompressor;
    std::thread decompressionThread;
    
    mutable std::mutex compressedFramesMutex;
    mutable std::mutex decompressedFramesMutex;
    std::condition_variable compressedFramesCV;
    std::condition_variable decompressedFramesCV;
    
    std::deque<std::vector<char>> compressedFrames;
    std::deque<std::vector<float>> decompressedFrames;
    
    std::atomic<bool> shouldStop;
}; 