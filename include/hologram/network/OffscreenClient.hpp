#pragma once

#include "network/BaseClient.hpp"
#include <chrono>

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
    double currentFPS;
    double avgRenderTime;
    
    void mockRendering(const std::vector<float>& vertices);
    void updatePerformanceMetrics();
    void displayMetrics();
}; 