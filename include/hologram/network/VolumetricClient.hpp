#pragma once

#include "hologram/network/NetworkProtocol.hpp"
#include "hologram/network/ReliableNetworkManager.hpp"
#include "hologram/network/BaseClient.hpp"
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <netinet/in.h>
#include <chrono>
#include <sstream>

namespace hologram {

class VolumetricClient : public BaseClient {
public:
    VolumetricClient(const std::string& serverIP, int serverPort);
    virtual ~VolumetricClient() override;

    void start();
    void stop();
    void run() override;

private:
    bool running;
    std::unique_ptr<ReliableNetworkManager> networkManager;
    
    // Network socket
    int sockfd;
    struct sockaddr_in serverAddr;
    
    // Client thread
    std::unique_ptr<std::thread> clientThread;
    void clientLoop();

    // Performance metrics
    int frameCount;
    double currentFPS;
    std::chrono::steady_clock::time_point lastFPSUpdate;
    std::chrono::steady_clock::time_point lastMetricsDisplay;
    std::stringstream metricsBuffer;
    static constexpr double METRICS_UPDATE_INTERVAL = 0.5; // seconds

    void updatePerformanceMetrics();
    void displayMetrics();
    void clearMetricsBuffer();
};

} // namespace hologram 