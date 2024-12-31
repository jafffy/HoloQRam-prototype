#include "network/DecompressionManager.hpp"
#include <sstream>
#include <iostream>

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
}

void DecompressionManager::addCompressedFrame(std::vector<char>&& compressedData) {
    {
        std::unique_lock<std::mutex> lock(compressedFramesMutex);
        while (compressedFrames.size() >= MAX_COMPRESSED_FRAMES && !shouldStop) {
            compressedFrames.pop_front();
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
    return true;
}

void DecompressionManager::decompressFrames() {
    while (!shouldStop) {
        std::vector<char> compressedData;
        
        // Get compressed frame
        {
            std::unique_lock<std::mutex> lock(compressedFramesMutex);
            compressedFramesCV.wait(lock, [this] {
                return !compressedFrames.empty() || shouldStop;
            });
            
            if (shouldStop) break;
            
            compressedData = std::move(compressedFrames.front());
            compressedFrames.pop_front();
        }
        
        try {
            // Decompress the frame
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            decompressor->decompress(compressedData, cloud);
            
            // Convert to vertices
            std::vector<float> vertices;
            vertices.reserve(cloud->points.size() * 6);
            for (const auto& point : cloud->points) {
                vertices.push_back(point.x);
                vertices.push_back(point.y);
                vertices.push_back(point.z);
                vertices.push_back(point.r / 255.0f);
                vertices.push_back(point.g / 255.0f);
                vertices.push_back(point.b / 255.0f);
            }
            
            // Store decompressed frame
            {
                std::unique_lock<std::mutex> lock(decompressedFramesMutex);
                while (decompressedFrames.size() >= MAX_DECOMPRESSED_FRAMES) {
                    decompressedFrames.pop_front();
                }
                decompressedFrames.push_back(std::move(vertices));
            }
            decompressedFramesCV.notify_one();
        } catch (const std::exception& e) {
            std::cerr << "Error decompressing frame: " << e.what() << std::endl;
        }
    }
} 