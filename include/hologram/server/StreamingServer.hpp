#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "hologram/compression/TemporalCompression.hpp"
#include "hologram/compression/VisibilityAwareCompression.hpp"
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>

namespace hologram {

class StreamingServer {
public:
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    
    struct ClientInfo {
        int socketFd;
        ViewportInfo viewport;
        std::chrono::steady_clock::time_point lastUpdate;
        std::unique_ptr<TemporalCompression> compressor;
    };
    
    struct Settings {
        uint16_t port;              // Server port
        size_t maxClients;          // Maximum number of clients
        float updateInterval;       // Minimum time between updates (seconds)
        bool enableCompression;     // Enable compression
        
        Settings()
            : port(8080)
            , maxClients(10)
            , updateInterval(0.033f)  // ~30 FPS
            , enableCompression(true) {}
    };
    
    explicit StreamingServer(const Settings& settings = Settings());
    ~StreamingServer();
    
    // Server control
    bool start();
    void stop();
    bool isRunning() const { return running; }
    
    // Point cloud management
    void updatePointCloud(const CloudT::ConstPtr& cloud);
    
    // Client management
    size_t getClientCount() const;
    std::vector<int> getConnectedClients() const;
    
    // Statistics
    struct Stats {
        size_t totalBytesSent;
        size_t totalBytesReceived;
        float averageUpdateTime;
        size_t activeClients;
        float compressionRatio;
    };
    Stats getStats() const;
    
private:
    Settings settings;
    std::atomic<bool> running;
    int serverSocket;
    
    // Client management
    std::unordered_map<int, ClientInfo> clients;
    mutable std::mutex clientsMutex;
    
    // Point cloud data
    CloudT::Ptr currentCloud;
    mutable std::mutex cloudMutex;
    
    // Network threads
    std::unique_ptr<std::thread> acceptThread;
    std::unique_ptr<std::thread> updateThread;
    std::vector<std::unique_ptr<std::thread>> clientThreads;
    
    // Message queues
    struct Message {
        int clientId;
        std::vector<char> data;
    };
    std::queue<Message> sendQueue;
    mutable std::mutex queueMutex;
    std::condition_variable queueCV;
    
    // Thread functions
    void acceptLoop();
    void updateLoop();
    void clientLoop(int clientId);
    
    // Client handling
    void handleViewportUpdate(int clientId, const char* data, size_t length);
    void sendCompressedCloud(int clientId, const std::vector<char>& data);
    void removeClient(int clientId);
    
    // Helper functions
    bool initializeSocket();
    void cleanupSocket();
    void processClientMessages(int clientId);
    
    // Statistics tracking
    Stats stats;
    mutable std::mutex statsMutex;
    void updateStats(size_t bytesSent, size_t bytesReceived);
};

} // namespace hologram 