#pragma once

#include <memory>
#include <atomic>
#include <vector>

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
    double lastFrameTime;
    int frameCount;
    double lastFPSUpdate;
    double currentFPS;
    double avgRenderTime;
    
    void mockRendering(const std::vector<float>& vertices);
    void updatePerformanceMetrics();
    void displayMetrics();
}; 