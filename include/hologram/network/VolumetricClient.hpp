#pragma once

#include "hologram/network/NetworkProtocol.hpp"
#include "hologram/network/ReliableNetworkManager.hpp"
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <netinet/in.h>

namespace hologram {

class VolumetricClient {
public:
    VolumetricClient(const std::string& serverIP, int serverPort);
    ~VolumetricClient();

    void start();
    void stop();

private:
    bool running;
    std::unique_ptr<ReliableNetworkManager> networkManager;
    
    // Network socket
    int sockfd;
    struct sockaddr_in serverAddr;
    
    // Client thread
    std::unique_ptr<std::thread> clientThread;
    void clientLoop();
};

} // namespace hologram 