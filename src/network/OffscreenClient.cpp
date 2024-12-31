#include "hologram/network/OffscreenClient.hpp"
#include "hologram/network/NetworkManager.hpp"
#include "hologram/network/DecompressionManager.hpp"
#include "hologram/network/NetworkProtocol.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <random>
#include <sstream>

namespace hologram {

OffscreenClient::OffscreenClient(const std::string& compressionScheme,
                               const std::string& serverIP,
                               int serverPort)
    : BaseClient(compressionScheme)
    , running(true)
    , fps(0.0)
    , bandwidth(0.0)
    , latency(0.0)
    , lastMetricsUpdate(std::chrono::steady_clock::now())
    , lastFrameTime(std::chrono::steady_clock::now())
    , frameCount(0)
{
    // Create managers
    decompressionManager = std::make_unique<DecompressionManager>(compressionScheme);
    networkManager = std::make_unique<NetworkManager>(decompressionManager.get(), serverIP, serverPort);
    
    // Start managers
    networkManager->start();
    decompressionManager->start();
}

OffscreenClient::~OffscreenClient() {
    running = false;
    
    if (networkManager) {
        networkManager->stop();
    }
    if (decompressionManager) {
        decompressionManager->stop();
    }
}

void OffscreenClient::updateMetrics() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMetricsUpdate).count();
    
    if (elapsed >= METRICS_UPDATE_INTERVAL * 1000) {  // Convert to milliseconds
        // Update FPS based on actual frame count
        fps = static_cast<double>(frameCount) / (elapsed / 1000.0);
        
        std::cout << "[OffscreenClient::updateMetrics] Stats - "
                  << "Duration: " << elapsed << "ms, "
                  << "Frame count: " << frameCount << ", "
                  << "FPS: " << fps << std::endl;
        
        frameCount = 0;  // Reset frame counter
        
        // Get network metrics
        if (networkManager) {
            bandwidth = networkManager->getCurrentBandwidth();
            latency = networkManager->getCurrentRTT();
        }
        
        lastMetricsUpdate = now;
    }
}

void OffscreenClient::displayMetrics() {
    std::cout << "\r[Metrics] FPS: " << std::fixed << std::setprecision(1) << fps
              << " | Bandwidth: " << std::setprecision(2) << bandwidth << " MB/s"
              << " | Latency: " << std::setprecision(1) << latency << " ms"
              << std::flush;
}

void OffscreenClient::run() {
    // Create initial viewport info
    ViewportInfo viewport;
    viewport.position[0] = 0.0f;
    viewport.position[1] = 0.0f;
    viewport.position[2] = 0.0f;
    viewport.rotation[0] = 0.0f;
    viewport.rotation[1] = 0.0f;
    viewport.rotation[2] = 0.0f;
    viewport.fov = 60.0f;
    viewport.aspectRatio = 16.0f / 9.0f;
    viewport.nearPlane = 0.1f;
    viewport.farPlane = 100.0f;

    int failedAttempts = 0;
    const int MAX_FAILED_ATTEMPTS = 5;

    while (running && !shouldStop) {
        try {
            auto now = std::chrono::steady_clock::now();
            
            // Send viewport update
            if (!networkManager->sendViewportUpdate(viewport)) {
                std::cerr << "Failed to send viewport update" << std::endl;
                failedAttempts++;
                if (failedAttempts >= MAX_FAILED_ATTEMPTS) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    failedAttempts = 0;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }
            failedAttempts = 0;

            // Count this as a frame since we successfully sent a viewport update
            frameCount++;

            // Try to get next frame (point cloud data)
            auto frame = getNextFrame();

            // Update metrics
            updateMetrics();
            displayMetrics();

            // Calculate time spent in this frame
            auto frameEnd = std::chrono::steady_clock::now();
            auto frameTime = std::chrono::duration_cast<std::chrono::milliseconds>(frameEnd - now).count();
            
            // Sleep for remaining time to maintain target frame rate
            int targetFrameTime = 33;  // ~30 FPS
            if (frameTime < targetFrameTime) {
                std::this_thread::sleep_for(std::chrono::milliseconds(targetFrameTime - frameTime));
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in run loop: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr OffscreenClient::getNextFrame() {
    if (!decompressionManager) {
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    if (decompressionManager->getDecompressedCloud(cloud)) {
        return cloud;
    }
    return nullptr;
}

} // namespace hologram 