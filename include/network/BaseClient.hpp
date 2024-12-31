#pragma once

#include <memory>
#include <atomic>
#include <vector>

// Forward declarations
class NetworkManager;
class DecompressionManager;

class BaseClient {
public:
    BaseClient();
    virtual ~BaseClient();
    
    virtual void run() = 0;

protected:
    // Core components
    std::unique_ptr<NetworkManager> networkManager;
    std::unique_ptr<DecompressionManager> decompressionManager;
    
    std::atomic<bool> shouldStop;
    
    // Common functionality
    bool getNextFrame(std::vector<float>& currentVertices);
    void initializeNetworking();
    void cleanupNetworking();
}; 