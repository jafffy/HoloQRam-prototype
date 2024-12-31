#pragma once

#include <memory>
#include <atomic>
#include <vector>
#include <sstream>
#include <random>
#include <string>

// Forward declarations
class NetworkManager;
class DecompressionManager;

class OffscreenClient {
public:
    explicit OffscreenClient(const std::string& compressionScheme, 
                           const std::string& serverIP = "127.0.0.1",
                           int serverPort = 8766);
    ~OffscreenClient();
    
    void run();

private:
    // Core components
    std::unique_ptr<NetworkManager> networkManager;
    std::unique_ptr<DecompressionManager> decompressionManager;
    std::string compressionScheme;
    std::string serverIP;
    int serverPort;
    
    std::atomic<bool> shouldStop;
    
    // Performance metrics
    double lastFrameTime;
    int frameCount;
    double lastFPSUpdate;
    double currentFPS;
    double avgRenderTime;
    std::chrono::steady_clock::time_point lastMetricsDisplay;
    std::stringstream metricsBuffer;
    
    // Random number generation for mock rendering
    std::mt19937 rng;
    std::uniform_real_distribution<double> renderTimeDistribution;
    
    static constexpr double METRICS_UPDATE_INTERVAL = 0.5; // seconds
    
    void mockRendering(const std::vector<float>& vertices);
    void updatePerformanceMetrics();
    void displayMetrics();
    void clearMetricsBuffer();
    bool getNextFrame(std::vector<float>& vertices);
}; 