#pragma once

// Standard includes
#include <memory>
#include <atomic>
#include <chrono>

// Boost includes (before PCL)
#include <boost/concept_check.hpp>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Project includes
#include "hologram/network/BaseClient.hpp"
#include "hologram/network/NetworkManager.hpp"
#include "hologram/network/DecompressionManager.hpp"

namespace hologram {

class OffscreenClient : public BaseClient {
public:
    OffscreenClient(const std::string& compressionScheme = "vivo",
                    const std::string& serverIP = "127.0.0.1",
                    int serverPort = 8765);
    ~OffscreenClient() override;

    void run() override;

private:
    std::unique_ptr<NetworkManager> networkManager;
    std::unique_ptr<DecompressionManager> decompressionManager;
    std::atomic<bool> running;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNextFrame();
    void updateMetrics();
    void displayMetrics();

    // Performance metrics
    double fps;
    double bandwidth;
    double latency;
    std::chrono::steady_clock::time_point lastMetricsUpdate;
    std::chrono::steady_clock::time_point lastFrameTime;
    int frameCount;
    static constexpr double METRICS_UPDATE_INTERVAL = 0.5; // seconds
};

} // namespace hologram 