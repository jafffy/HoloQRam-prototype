#include "hologram/network/VolumetricClient.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <thread>

namespace hologram {

VolumetricClient::VolumetricClient(const std::string& serverIP, int serverPort)
    : running(false)
    , sockfd(-1)
{
    // Initialize socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        throw std::runtime_error("Error creating socket");
    }

    // Set up server address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    if (inet_pton(AF_INET, serverIP.c_str(), &serverAddr.sin_addr) <= 0) {
        throw std::runtime_error("Invalid server IP address");
    }

    // Set socket buffer size
    int recvbuff = 1024 * 1024; // 1MB buffer
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &recvbuff, sizeof(recvbuff)) < 0) {
        std::cerr << "Warning: Could not set socket buffer size" << std::endl;
    }

    // Initialize network manager
    networkManager = std::make_unique<ReliableNetworkManager>(sockfd);
}

VolumetricClient::~VolumetricClient() {
    stop();
    if (sockfd >= 0) {
        close(sockfd);
    }
}

void VolumetricClient::start() {
    if (running) return;
    
    running = true;
    networkManager->start();
    clientThread = std::make_unique<std::thread>(&VolumetricClient::clientLoop, this);
}

void VolumetricClient::stop() {
    if (!running) return;
    
    running = false;
    networkManager->stop();
    
    if (clientThread && clientThread->joinable()) {
        clientThread->join();
    }
}

void VolumetricClient::clientLoop() {
    while (running) {
        // Send viewport update
        ViewportInfo viewport;
        viewport.position[0] = 0.0f;
        viewport.position[1] = 0.0f;
        viewport.position[2] = 0.0f;
        viewport.rotation[0] = 0.0f;
        viewport.rotation[1] = 0.0f;
        viewport.rotation[2] = 0.0f;
        viewport.fov = 60.0f;
        viewport.aspectRatio = 16.0f / 9.0f;
        viewport.nearPlane = 0.1f;
        viewport.farPlane = 100.0f;

        // Serialize viewport data
        std::vector<char> data(sizeof(ViewportInfo));
        std::memcpy(data.data(), &viewport, sizeof(ViewportInfo));
        
        // Send via network manager
        networkManager->sendReliable(data, PacketType::VIEWPORT_UPDATE);
        
        // Sleep for a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
}

} // namespace hologram 