#include "network/OffscreenClient.hpp"
#include "network/NetworkManager.hpp"
#include "network/DecompressionManager.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <random>

// Constant for maximum expected points in a frame
const size_t MAX_POINTS_PER_FRAME = 500000;  // Adjust based on your typical maximum point cloud size

OffscreenClient::OffscreenClient()
    : BaseClient()
    , frameCount(0)
    , currentFPS(0.0)
    , avgRenderTime(0.0)
    , rng(std::random_device{}())  // Initialize with true random seed
    , renderTimeDistribution(0.8, 1.2)  // ±20% variation
{
    lastFrameTime = std::chrono::steady_clock::now();
    lastFPSUpdate = std::chrono::steady_clock::now();
    lastMetricsDisplay = std::chrono::steady_clock::now();
    metricsBuffer.setf(std::ios::fixed, std::ios::floatfield);
    metricsBuffer.precision(2);
}

OffscreenClient::~OffscreenClient() {
    // Ensure proper cleanup
    shouldStop = true;
    clearMetricsBuffer();
}

void OffscreenClient::clearMetricsBuffer() {
    metricsBuffer.str("");
    metricsBuffer.clear();
}

void OffscreenClient::mockRendering(const std::vector<float>& vertices) {
    // Simulate rendering time based on point cloud size
    // More points = more rendering time
    int numPoints = vertices.size() / 6; // 6 floats per point (xyz + rgb)
    
    // Base rendering time: 2ms + 0.01ms per 1000 points
    double baseTime = 2.0 + (numPoints / 1000.0) * 0.01;
    
    // Add random variation (±20%) using member RNG
    double renderTime = baseTime * renderTimeDistribution(rng);
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
    auto currentTime = std::chrono::steady_clock::now();
    auto timeSinceLastDisplay = std::chrono::duration<double>(currentTime - lastMetricsDisplay).count();
    
    // Only update display at specified interval
    if (timeSinceLastDisplay < METRICS_UPDATE_INTERVAL) {
        return;
    }
    
    // Clear previous buffer content
    clearMetricsBuffer();
    
    // Build metrics string in buffer
    metricsBuffer << "\033[2J\033[H"  // Clear screen and move cursor to top
                 << "Performance Metrics:\n"
                 << "==================\n"
                 << "FPS: " << currentFPS << "\n"
                 << "Bandwidth: " << networkManager->getCurrentBandwidth() << " MB/s\n"
                 << "RTT: " << networkManager->getCurrentRTT() << " ms\n"
                 << "Avg Render Time: " << avgRenderTime << " ms\n"
                 << "Compressed Frames Queue: " << decompressionManager->getCompressedQueueSize() << "\n"
                 << "Decompressed Frames Queue: " << decompressionManager->getDecompressedQueueSize() << "\n"
                 << "\nPress Ctrl+C to exit\n";
    
    // Write buffer to output and flush
    std::cout << metricsBuffer.str() << std::flush;
    
    lastMetricsDisplay = currentTime;
}

void OffscreenClient::run() {
    // Pre-allocate vector with maximum expected size
    // 6 floats per point (xyz + rgb)
    std::vector<float> currentVertices;
    currentVertices.reserve(MAX_POINTS_PER_FRAME * 6);

    while (!shouldStop) {
        // Clear the vector while maintaining capacity
        currentVertices.clear();
        
        // Get next frame if available
        if (getNextFrame(currentVertices)) {
            // Check if frame size is within reasonable limits
            if (currentVertices.size() > MAX_POINTS_PER_FRAME * 6) {
                std::cerr << "Warning: Received oversized frame with " 
                         << (currentVertices.size() / 6) << " points" << std::endl;
            }
            
            // Mock rendering with sleep
            mockRendering(currentVertices);
        }
        
        updatePerformanceMetrics();
        displayMetrics();
        
        // Cap the update rate to ~60Hz to avoid excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    
    // Explicitly free memory at shutdown
    std::vector<float>().swap(currentVertices);
} 