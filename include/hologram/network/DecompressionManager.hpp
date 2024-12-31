#pragma once

#include "hologram/compression/CompressionScheme.hpp"
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

class DecompressionManager {
public:
    explicit DecompressionManager(const std::string& compressionScheme);
    ~DecompressionManager();

    void start();
    void stop();

    void addCompressedData(const std::vector<char>& data);
    bool getDecompressedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    // Queue size getters
    size_t getCompressedQueueSize() const {
        std::lock_guard<std::mutex> lock(queueMutex);
        return compressedQueue.size();
    }

    size_t getDecompressedQueueSize() const {
        std::lock_guard<std::mutex> lock(cloudMutex);
        return decompressedQueue.size();
    }

private:
    bool running;
    std::unique_ptr<hologram::CompressionScheme> decompressor;

    // Thread for decompression
    std::unique_ptr<std::thread> decompThread;
    void decompressFrames();

    // Queue for compressed data
    std::queue<std::vector<char>> compressedQueue;
    mutable std::mutex queueMutex;
    std::condition_variable queueCV;

    // Queue for decompressed clouds
    std::queue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> decompressedQueue;
    mutable std::mutex cloudMutex;
    std::condition_variable cloudCV;

    static constexpr size_t MAX_QUEUE_SIZE = 30;
}; 