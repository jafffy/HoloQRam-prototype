#include "network/OffscreenClient.hpp"
#include "network/NetworkManager.hpp"
#include "network/DecompressionManager.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <random>

OffscreenClient::OffscreenClient()
    : BaseClient()
    , frameCount(0)
    , currentFPS(0.0)
    , avgRenderTime(0.0)
{
    lastFrameTime = std::chrono::steady_clock::now();
    lastFPSUpdate = std::chrono::steady_clock::now();
}

OffscreenClient::~OffscreenClient() = default;

void OffscreenClient::mockRendering(const std::vector<float>& vertices) {
    // Simulate rendering time based on point cloud size
    // More points = more rendering time
    int numPoints = vertices.size() / 6; // 6 floats per point (xyz + rgb)
    
    // Base rendering time: 2ms + 0.01ms per 1000 points
    double baseTime = 2.0 + (numPoints / 1000.0) * 0.01;
    
    // Add some random variation (±20%)
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.8, 1.2);
    
    double renderTime = baseTime * dis(gen);
    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(renderTime * 1000)));
    
    // Update average render time (simple moving average)
    avgRenderTime = avgRenderTime * 0.95 + renderTime * 0.05;
}

void OffscreenClient::updatePerformanceMetrics() {
    auto currentTime = std::chrono::steady_clock::now();
    frameCount++;

    // Update FPS every second
    auto timeSinceLastUpdate = std::chrono::duration<double>(currentTime - lastFPSUpdate).count();
    if (timeSinceLastUpdate >= 1.0) {
        currentFPS = frameCount / timeSinceLastUpdate;
        frameCount = 0;
        lastFPSUpdate = currentTime;
    }
}

void OffscreenClient::displayMetrics() {
    std::cout << "\033[2J\033[H";  // Clear screen and move cursor to top
    std::cout << std::fixed << std::setprecision(2)
              << "Performance Metrics:\n"
              << "==================\n"
              << "FPS: " << currentFPS << "\n"
              << "Bandwidth: " << networkManager->getCurrentBandwidth() << " MB/s\n"
              << "RTT: " << networkManager->getCurrentRTT() << " ms\n"
              << "Avg Render Time: " << avgRenderTime << " ms\n"
              << "Compressed Frames Queue: " << decompressionManager->getCompressedQueueSize() << "\n"
              << "Decompressed Frames Queue: " << decompressionManager->getDecompressedQueueSize() << "\n"
              << "\nPress Ctrl+C to exit\n";
}

void OffscreenClient::run() {
    std::vector<float> currentVertices;

    while (!shouldStop) {
        // Get next frame if available
        if (getNextFrame(currentVertices)) {
            // Mock rendering with sleep
            mockRendering(currentVertices);
        }
        
        updatePerformanceMetrics();
        displayMetrics();
        
        // Cap the update rate to ~60Hz to avoid excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
} 