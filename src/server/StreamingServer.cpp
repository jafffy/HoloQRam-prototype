#include "hologram/server/StreamingServer.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

namespace hologram {

StreamingServer::StreamingServer(const Settings& settings)
    : settings(settings)
    , running(false)
    , serverSocket(-1)
    , currentCloud(new CloudT)
    , stats{0, 0, 0.0f, 0, 0.0f} {
}

StreamingServer::~StreamingServer() {
    stop();
}

bool StreamingServer::start() {
    if (running) return false;
    
    if (!initializeSocket()) {
        return false;
    }
    
    running = true;
    
    // Start server threads
    acceptThread = std::make_unique<std::thread>(&StreamingServer::acceptLoop, this);
    updateThread = std::make_unique<std::thread>(&StreamingServer::updateLoop, this);
    
    return true;
}

void StreamingServer::stop() {
    if (!running) return;
    
    running = false;
    queueCV.notify_all();
    
    // Stop accept thread
    if (acceptThread && acceptThread->joinable()) {
        acceptThread->join();
    }
    
    // Stop update thread
    if (updateThread && updateThread->joinable()) {
        updateThread->join();
    }
    
    // Stop client threads
    {
        std::lock_guard<std::mutex> lock(clientsMutex);
        for (auto& [clientId, info] : clients) {
            close(info.socketFd);
        }
        clients.clear();
    }
    
    // Cleanup client threads
    for (auto& thread : clientThreads) {
        if (thread && thread->joinable()) {
            thread->join();
        }
    }
    clientThreads.clear();
    
    cleanupSocket();
}

bool StreamingServer::initializeSocket() {
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        return false;
    }
    
    // Set socket options
    int opt = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        cleanupSocket();
        return false;
    }
    
    // Set non-blocking mode
    int flags = fcntl(serverSocket, F_GETFL, 0);
    if (flags < 0 || fcntl(serverSocket, F_SETFL, flags | O_NONBLOCK) < 0) {
        cleanupSocket();
        return false;
    }
    
    // Bind socket
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(settings.port);
    
    if (bind(serverSocket, (struct sockaddr*)&address, sizeof(address)) < 0) {
        cleanupSocket();
        return false;
    }
    
    // Listen for connections
    if (listen(serverSocket, settings.maxClients) < 0) {
        cleanupSocket();
        return false;
    }
    
    return true;
}

void StreamingServer::cleanupSocket() {
    if (serverSocket >= 0) {
        close(serverSocket);
        serverSocket = -1;
    }
}

void StreamingServer::acceptLoop() {
    while (running) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientSocket < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                // No pending connections
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // Error handling
            continue;
        }
        
        // Set client socket to non-blocking
        int flags = fcntl(clientSocket, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(clientSocket, F_SETFL, flags | O_NONBLOCK);
        }
        
        // Add client
        {
            std::lock_guard<std::mutex> lock(clientsMutex);
            if (clients.size() >= settings.maxClients) {
                close(clientSocket);
                continue;
            }
            
            ClientInfo info;
            info.socketFd = clientSocket;
            info.lastUpdate = std::chrono::steady_clock::now();
            info.compressor = std::make_unique<TemporalCompression>();
            
            clients[clientSocket] = std::move(info);
            
            // Start client thread
            clientThreads.push_back(
                std::make_unique<std::thread>(&StreamingServer::clientLoop, this, clientSocket)
            );
        }
    }
}

void StreamingServer::updateLoop() {
    while (running) {
        auto startTime = std::chrono::steady_clock::now();
        
        // Process point cloud updates
        CloudT::Ptr cloud;
        {
            std::lock_guard<std::mutex> lock(cloudMutex);
            cloud = currentCloud;
        }
        
        if (!cloud->empty()) {
            std::lock_guard<std::mutex> lock(clientsMutex);
            for (auto& [clientId, info] : clients) {
                auto now = std::chrono::steady_clock::now();
                float timeSinceUpdate = std::chrono::duration<float>(
                    now - info.lastUpdate).count();
                
                if (timeSinceUpdate >= settings.updateInterval) {
                    // Compress point cloud for this client's viewport
                    auto compressedFrame = info.compressor->compressNextFrame(cloud, info.viewport);
                    
                    // Queue compressed data for sending
                    {
                        std::lock_guard<std::mutex> qLock(queueMutex);
                        sendQueue.push({clientId, std::move(compressedFrame.data)});
                    }
                    queueCV.notify_one();
                    
                    info.lastUpdate = now;
                }
            }
        }
        
        // Update statistics
        auto endTime = std::chrono::steady_clock::now();
        float updateTime = std::chrono::duration<float>(endTime - startTime).count();
        
        {
            std::lock_guard<std::mutex> lock(statsMutex);
            stats.averageUpdateTime = updateTime;
            stats.activeClients = clients.size();
        }
        
        // Sleep to maintain update interval
        std::this_thread::sleep_for(
            std::chrono::duration<float>(settings.updateInterval));
    }
}

void StreamingServer::clientLoop(int clientId) {
    while (running) {
        // Process incoming messages
        processClientMessages(clientId);
        
        // Send queued messages
        Message msg;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            auto hasMessage = queueCV.wait_for(lock, 
                std::chrono::milliseconds(100),
                [this, clientId]() {
                    return !running || 
                           (!sendQueue.empty() && sendQueue.front().clientId == clientId);
                });
                
            if (!hasMessage || !running) continue;
            
            msg = std::move(sendQueue.front());
            sendQueue.pop();
        }
        
        sendCompressedCloud(clientId, msg.data);
    }
}

void StreamingServer::processClientMessages(int clientId) {
    std::lock_guard<std::mutex> lock(clientsMutex);
    auto it = clients.find(clientId);
    if (it == clients.end()) return;
    
    char buffer[4096];
    ssize_t bytesRead;
    
    while ((bytesRead = recv(it->second.socketFd, buffer, sizeof(buffer), 0)) > 0) {
        updateStats(0, bytesRead);
        handleViewportUpdate(clientId, buffer, bytesRead);
    }
    
    if (bytesRead < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
            removeClient(clientId);
        }
    } else if (bytesRead == 0) {
        // Client disconnected
        removeClient(clientId);
    }
}

void StreamingServer::handleViewportUpdate(int clientId, const char* data, size_t length) {
    if (length < sizeof(ViewportInfo)) return;
    
    const ViewportInfo* viewport = reinterpret_cast<const ViewportInfo*>(data);
    
    std::lock_guard<std::mutex> lock(clientsMutex);
    auto it = clients.find(clientId);
    if (it != clients.end()) {
        it->second.viewport = *viewport;
    }
}

void StreamingServer::sendCompressedCloud(int clientId, const std::vector<char>& data) {
    std::lock_guard<std::mutex> lock(clientsMutex);
    auto it = clients.find(clientId);
    if (it == clients.end()) return;
    
    ssize_t bytesSent = send(it->second.socketFd, data.data(), data.size(), 0);
    if (bytesSent > 0) {
        updateStats(bytesSent, 0);
    }
}

void StreamingServer::removeClient(int clientId) {
    std::lock_guard<std::mutex> lock(clientsMutex);
    auto it = clients.find(clientId);
    if (it != clients.end()) {
        close(it->second.socketFd);
        clients.erase(it);
    }
}

void StreamingServer::updatePointCloud(const CloudT::ConstPtr& cloud) {
    std::lock_guard<std::mutex> lock(cloudMutex);
    *currentCloud = *cloud;
}

size_t StreamingServer::getClientCount() const {
    std::lock_guard<std::mutex> lock(clientsMutex);
    return clients.size();
}

std::vector<int> StreamingServer::getConnectedClients() const {
    std::lock_guard<std::mutex> lock(clientsMutex);
    std::vector<int> clientIds;
    clientIds.reserve(clients.size());
    for (const auto& [id, _] : clients) {
        clientIds.push_back(id);
    }
    return clientIds;
}

StreamingServer::Stats StreamingServer::getStats() const {
    std::lock_guard<std::mutex> lock(statsMutex);
    return stats;
}

void StreamingServer::updateStats(size_t bytesSent, size_t bytesReceived) {
    std::lock_guard<std::mutex> lock(statsMutex);
    stats.totalBytesSent += bytesSent;
    stats.totalBytesReceived += bytesReceived;
    
    // Update compression ratio
    if (bytesSent > 0) {
        float ratio = static_cast<float>(currentCloud->size() * sizeof(PointT)) /
                     static_cast<float>(bytesSent);
        stats.compressionRatio = (stats.compressionRatio * 0.9f) + (ratio * 0.1f);
    }
}

} // namespace hologram 