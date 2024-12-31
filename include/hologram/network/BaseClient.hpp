#pragma once

// Standard includes
#include <memory>
#include <atomic>
#include <vector>
#include <string>

namespace hologram {

// Forward declarations
class NetworkManager;
class DecompressionManager;

class BaseClient {
public:
    BaseClient(const std::string& compressionScheme = "vivo");
    virtual ~BaseClient();
    
    virtual void run() = 0;

protected:
    // Core components
    std::unique_ptr<NetworkManager> networkManager;
    std::unique_ptr<DecompressionManager> decompressionManager;
    
    std::atomic<bool> shouldStop;
    std::string compressionScheme;
    
    // Common functionality
    bool getNextFrame(std::vector<float>& currentVertices);
    void initializeNetworking();
    void cleanupNetworking();
};

} // namespace hologram 