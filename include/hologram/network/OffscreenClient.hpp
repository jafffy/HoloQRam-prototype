#pragma once

#include "network/BaseClient.hpp"
#include <chrono>
#include <random>
#include <sstream>

class OffscreenClient : public BaseClient {
public:
    OffscreenClient();
    ~OffscreenClient() override;
    
    void run() override;

private:
    // Performance metrics
    std::chrono::steady_clock::time_point lastFrameTime;
    int frameCount;
    std::chrono::steady_clock::time_point lastFPSUpdate;
    std::chrono::steady_clock::time_point lastMetricsDisplay;
    double currentFPS;
    double avgRenderTime;
    
    // Random number generation for mock rendering
    std::mt19937 rng;
    std::uniform_real_distribution<double> renderTimeDistribution;
    
    // Buffer for metrics display
    std::stringstream metricsBuffer;
    static constexpr double METRICS_UPDATE_INTERVAL = 0.25; // Update display 4 times per second
    
    void mockRendering(const std::vector<float>& vertices);
    void updatePerformanceMetrics();
    void displayMetrics();
    void clearMetricsBuffer();
}; 