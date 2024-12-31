#include "hologram/network/DecompressionManager.hpp"
#include <sstream>
#include <thread>
#include <chrono>

// Reduce maximum queue sizes to prevent memory buildup
const size_t MAX_COMPRESSED_FRAMES = 5;    // Reduced from previous value
const size_t MAX_DECOMPRESSED_FRAMES = 2;  // Reduced from previous value

DecompressionManager::DecompressionManager(const std::string& compressionScheme)
    : decompressor(CompressionScheme::create(compressionScheme))
    , shouldStop(false)
{
}

DecompressionManager::~DecompressionManager() {
    stop();
}

void DecompressionManager::start() {
    shouldStop = false;
    decompressionThread = std::thread(&DecompressionManager::decompressFrames, this);
}

void DecompressionManager::stop() {
    shouldStop = true;
    compressedFramesCV.notify_all();
    decompressedFramesCV.notify_all();
    
    if (decompressionThread.joinable()) {
        decompressionThread.join();
    }
    
    // Clear queues to free memory
    {
        std::unique_lock<std::mutex> lock(compressedFramesMutex);
        compressedFrames.clear();
    }
    {
        std::unique_lock<std::mutex> lock(decompressedFramesMutex);
        decompressedFrames.clear();
    }
}

void DecompressionManager::addCompressedFrame(std::vector<char>&& compressedData) {
    {
        std::unique_lock<std::mutex> lock(compressedFramesMutex);
        while (compressedFrames.size() >= MAX_COMPRESSED_FRAMES && !shouldStop) {
            // If queue is full, remove oldest frame to prevent memory buildup
            if (compressedFrames.size() >= MAX_COMPRESSED_FRAMES) {
                compressedFrames.pop_front();
            }
        }
        compressedFrames.push_back(std::move(compressedData));
    }
    compressedFramesCV.notify_one();
}

bool DecompressionManager::getNextDecompressedFrame(std::vector<float>& vertices) {
    std::unique_lock<std::mutex> lock(decompressedFramesMutex);
    if (decompressedFrames.empty()) {
        return false;
    }
    
    vertices = std::move(decompressedFrames.front());
    decompressedFrames.pop_front();
    decompressedFramesCV.notify_one();
    return true;
}

void DecompressionManager::decompressFrames() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    while (!shouldStop) {
        std::vector<char> compressedData;
        {
            std::unique_lock<std::mutex> lock(compressedFramesMutex);
            compressedFramesCV.wait(lock, [this]() {
                return !compressedFrames.empty() || shouldStop;
            });
            
            if (shouldStop) {
                break;
            }
            
            compressedData = std::move(compressedFrames.front());
            compressedFrames.pop_front();
        }
        compressedFramesCV.notify_one();
        
        try {
            // Decompress the data
            decompressor->decompress(compressedData, cloud);
            
            // Convert to vertices
            std::vector<float> vertices;
            vertices.reserve(cloud->points.size() * 6); // xyz + rgb
            
            for (const auto& point : cloud->points) {
                vertices.push_back(point.x);
                vertices.push_back(point.y);
                vertices.push_back(point.z);
                vertices.push_back(point.r / 255.0f);
                vertices.push_back(point.g / 255.0f);
                vertices.push_back(point.b / 255.0f);
            }
            
            {
                std::unique_lock<std::mutex> lock(decompressedFramesMutex);
                // If queue is full, remove oldest frame
                if (decompressedFrames.size() >= MAX_DECOMPRESSED_FRAMES) {
                    decompressedFrames.pop_front();
                }
                decompressedFrames.push_back(std::move(vertices));
            }
            decompressedFramesCV.notify_one();
            
            // Clear point cloud data but keep memory allocated
            cloud->points.clear();
            
        } catch (const std::exception& e) {
            std::cerr << "Error decompressing frame: " << e.what() << std::endl;
            // Continue processing next frame
            continue;
        }
    }
} 