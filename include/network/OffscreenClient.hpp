#pragma once

#include <memory>
#include <atomic>
#include <vector>
#include <chrono>

// Forward declarations
class NetworkManager;
class DecompressionManager;

class OffscreenClient {
public:
    OffscreenClient();
    ~OffscreenClient();
    
    void run();

private:
    // Core components
    std::unique_ptr<NetworkManager> networkManager;
    std::unique_ptr<DecompressionManager> decompressionManager;
    
    std::atomic<bool> shouldStop;
    
    // Performance metrics
    std::chrono::steady_clock::time_point lastFrameTime;
    int frameCount;
    std::chrono::steady_clock::time_point lastFPSUpdate;
    double currentFPS;
    double avgRenderTime;
    
    void mockRendering(const std::vector<float>& vertices);
    void updatePerformanceMetrics();
    void displayMetrics();
}; 