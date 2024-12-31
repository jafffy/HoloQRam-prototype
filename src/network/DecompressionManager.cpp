#include "hologram/network/DecompressionManager.hpp"
#include "hologram/compression/CompressionScheme.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>
#include <thread>
#include <chrono>
#include <iostream>

namespace hologram {

// Constants for queue sizes
static constexpr size_t MAX_QUEUE_SIZE = 5;

DecompressionManager::DecompressionManager(const std::string& compressionScheme)
    : running(false)
    , decompressor(CompressionScheme::create(compressionScheme))
{
    if (!decompressor) {
        throw std::runtime_error("Failed to create compression scheme: " + compressionScheme);
    }
}

DecompressionManager::~DecompressionManager() {
    stop();
}

void DecompressionManager::start() {
    if (running) return;
    running = true;
    decompThread = std::make_unique<std::thread>(&DecompressionManager::decompressFrames, this);
}

void DecompressionManager::stop() {
    running = false;
    queueCV.notify_all();
    cloudCV.notify_all();
    
    if (decompThread && decompThread->joinable()) {
        decompThread->join();
    }
    
    // Clear queues
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        while (!compressedQueue.empty()) {
            compressedQueue.pop();
        }
    }
    {
        std::lock_guard<std::mutex> lock(cloudMutex);
        while (!decompressedQueue.empty()) {
            decompressedQueue.pop();
        }
    }
}

void DecompressionManager::addCompressedData(const std::vector<char>& data) {
    std::lock_guard<std::mutex> lock(queueMutex);
    if (compressedQueue.size() < MAX_QUEUE_SIZE) {
        compressedQueue.push(data);
        queueCV.notify_one();
    }
}

bool DecompressionManager::getDecompressedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    std::lock_guard<std::mutex> lock(cloudMutex);
    if (decompressedQueue.empty()) {
        return false;
    }
    
    cloud = decompressedQueue.front();
    decompressedQueue.pop();
    return true;
}

void DecompressionManager::decompressFrames() {
    while (running) {
        std::vector<char> compressedData;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueCV.wait(lock, [this]() {
                return !compressedQueue.empty() || !running;
            });
            
            if (!running) {
                break;
            }
            
            compressedData = std::move(compressedQueue.front());
            compressedQueue.pop();
        }
        
        try {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            decompressor->decompress(compressedData, cloud);
            
            {
                std::lock_guard<std::mutex> lock(cloudMutex);
                if (decompressedQueue.size() < MAX_QUEUE_SIZE) {
                    decompressedQueue.push(cloud);
                    cloudCV.notify_one();
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error decompressing frame: " << e.what() << std::endl;
            continue;
        }
    }
}

} // namespace hologram 