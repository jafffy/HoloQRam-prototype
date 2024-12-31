#include "hologram/server/VolumetricServer.hpp"
#include "hologram/compression/ViVoCompression.hpp"
#include "hologram/network/NetworkProtocol.hpp"
#include "hologram/server/CellManager.hpp"

#include <pcl/io/pcd_io.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <sstream>
#include <cstring>

#define PORT 8765
#define MAX_PACKET_SIZE 65507 // Max UDP packet size
#define CHUNK_HEADER_SIZE 16   // 4 bytes each for: total chunks, chunk index, packet type, cell ID

namespace hologram {

VolumetricServer::VolumetricServer(const std::string& compressionScheme, float cellSize, bool verbose)
    : running(false)
    , verbose(verbose)
    , sockfd(-1)
    , compressor(CompressionScheme::create(compressionScheme))
    , cellManager(std::make_unique<CellManager>(cellSize)) {
    if (verbose) {
        std::cout << "Initializing VolumetricServer with:\n"
                  << "  Compression: " << compressionScheme << "\n"
                  << "  Cell size: " << cellSize << std::endl;
    }
    setupSocket();
    generateSamplePointCloud();
    cellManager->segmentPointCloud(cloud);
}

VolumetricServer::~VolumetricServer() {
    stop();
    close(sockfd);
}

void VolumetricServer::start() {
    if (running) {
        if (verbose) {
            std::cout << "[VolumetricServer::start] Server already running" << std::endl;
        }
        return;
    }
    running = true;
    if (verbose) {
        std::cout << "[VolumetricServer::start] Starting server thread..." << std::endl;
    }
    serverThread = std::make_unique<std::thread>(&VolumetricServer::serverLoop, this);
    if (verbose) {
        std::cout << "[VolumetricServer::start] Server thread started with ID: " 
                  << serverThread->get_id() << std::endl;
    }
}

void VolumetricServer::stop() {
    if (!running) {
        if (verbose) {
            std::cout << "[VolumetricServer::stop] Server already stopped" << std::endl;
        }
        return;
    }
    if (verbose) {
        std::cout << "[VolumetricServer::stop] Stopping server..." << std::endl;
    }
    running = false;
    if (serverThread && serverThread->joinable()) {
        if (verbose) {
            std::cout << "[VolumetricServer::stop] Waiting for server thread to join..." << std::endl;
        }
        serverThread->join();
        if (verbose) {
            std::cout << "[VolumetricServer::stop] Server thread joined" << std::endl;
        }
    }
}

void VolumetricServer::serverLoop() {
    if (verbose) {
        std::cout << "[VolumetricServer::serverLoop] Server thread starting on thread ID: " 
                  << std::this_thread::get_id() << std::endl;
        std::cout << "[VolumetricServer::serverLoop] Server starting on port " << PORT 
                  << " with socket fd: " << sockfd << std::endl;
    }

    char buffer[MAX_PACKET_SIZE];
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    size_t totalBytesReceived = 0;
    size_t packetCount = 0;
    auto lastStatsTime = std::chrono::steady_clock::now();

    if (verbose) {
        std::cout << "[VolumetricServer::serverLoop] Server starting on port " << PORT << std::endl;
    }

    while (running) {
        ssize_t bytesRead = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                                   (struct sockaddr*)&clientAddr, &clientAddrLen);
        
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastStatsTime).count();
        
        if (bytesRead < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Print stats every second if we're not receiving data
                if (duration >= 1000 && verbose) {
                    std::cout << "[VolumetricServer::serverLoop] No data received in the last second. "
                              << "Total packets: " << packetCount << ", "
                              << "Total bytes: " << totalBytesReceived << std::endl;
                    lastStatsTime = now;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            std::cerr << "[VolumetricServer::serverLoop] Error receiving data: " 
                      << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            continue;
        }

        totalBytesReceived += bytesRead;
        packetCount++;

        if (verbose) {
            std::cout << "[VolumetricServer::serverLoop] Received " << bytesRead << " bytes from "
                      << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << std::endl;
        }

        if (bytesRead >= sizeof(PacketHeader)) {
            const PacketHeader* header = reinterpret_cast<const PacketHeader*>(buffer);
            if (verbose) {
                std::cout << "[VolumetricServer::serverLoop] Packet details:\n"
                          << "  Type: " << static_cast<int>(header->type) << "\n"
                          << "  Sequence: " << header->sequenceNumber << "\n"
                          << "  Timestamp: " << header->timestamp << "\n"
                          << "  Total chunks: " << header->totalChunks << "\n"
                          << "  Chunk index: " << header->chunkIndex << "\n"
                          << "  Payload size: " << header->payloadSize << std::endl;
            }

            switch (header->type) {
                case PacketType::VIEWPORT_UPDATE:
                    if (verbose) {
                        std::cout << "[VolumetricServer::serverLoop] Processing viewport update..." << std::endl;
                    }
                    handleViewportUpdate(buffer + sizeof(PacketHeader), 
                                      bytesRead - sizeof(PacketHeader),
                                      clientAddr);
                    break;
                default:
                    std::cerr << "[VolumetricServer::serverLoop] Unknown packet type: " 
                             << static_cast<int>(header->type) << std::endl;
                    break;
            }
        } else {
            std::cerr << "[VolumetricServer::serverLoop] Received packet too small: " 
                      << bytesRead << " bytes" << std::endl;
        }

        // Print stats every second
        if (duration >= 1000 && verbose) {
            double bytesPerSecond = static_cast<double>(totalBytesReceived) / 
                                  (duration / 1000.0);
            std::cout << "[VolumetricServer::serverLoop] Statistics:\n"
                      << "  Duration: " << duration << "ms\n"
                      << "  Packets received: " << packetCount << "\n"
                      << "  Bytes received: " << totalBytesReceived << "\n"
                      << "  Bytes/second: " << bytesPerSecond << std::endl;
            
            totalBytesReceived = 0;
            packetCount = 0;
            lastStatsTime = now;
        }
    }
}

void VolumetricServer::handleViewportUpdate(const char* data, size_t length, const struct sockaddr_in& clientAddr) {
    std::cout << "[VolumetricServer::handleViewportUpdate] Received viewport update from "
              << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port)
              << " (size: " << length << " bytes)" << std::endl;

    if (length < sizeof(ViewportInfo)) {
        std::cerr << "[VolumetricServer::handleViewportUpdate] Received viewport update is too small: " 
                  << length << " bytes, expected " << sizeof(ViewportInfo) << " bytes" << std::endl;
        return;
    }

    const ViewportInfo* viewport = reinterpret_cast<const ViewportInfo*>(data);
    std::string clientKey = getClientKey(clientAddr);
    
    // Convert ViewportInfo to ViewportData
    ViewportData viewportData;
    viewportData.position = glm::vec3(viewport->position[0], viewport->position[1], viewport->position[2]);
    
    // Convert rotation angles to direction vector
    float pitch = viewport->rotation[0];  // x-axis rotation
    float yaw = viewport->rotation[1];    // y-axis rotation
    float roll = viewport->rotation[2];   // z-axis rotation (not used for direction)
    
    // Calculate direction vector from pitch and yaw
    viewportData.direction = glm::vec3(
        sin(yaw) * cos(pitch),
        sin(pitch),
        -cos(yaw) * cos(pitch)
    );
    
    viewportData.fov = viewport->fov;
    viewportData.aspectRatio = viewport->aspectRatio;
    viewportData.nearPlane = viewport->nearPlane;
    viewportData.farPlane = viewport->farPlane;
    
    std::cout << "[VolumetricServer::handleViewportUpdate] Viewport details for " << clientKey << ":\n"
              << "  Position: [" << viewportData.position.x << ", " 
              << viewportData.position.y << ", " 
              << viewportData.position.z << "]\n"
              << "  Direction: [" << viewportData.direction.x << ", " 
              << viewportData.direction.y << ", " 
              << viewportData.direction.z << "]\n"
              << "  FOV: " << viewportData.fov << ", Aspect Ratio: " << viewportData.aspectRatio << "\n"
              << "  Near/Far Planes: " << viewportData.nearPlane << "/" << viewportData.farPlane 
              << std::endl;

    clientViewports[clientKey] = viewportData;

    // Update cell visibility for the new viewport
    std::cout << "[VolumetricServer::handleViewportUpdate] Updating cell visibility..." << std::endl;
    cellManager->updateCellVisibility(viewportData);
    
    // Stream visible cells immediately for this client
    std::cout << "[VolumetricServer::handleViewportUpdate] Starting to stream visible cells..." << std::endl;
    streamVisibleCells(viewportData, clientAddr);
}

void VolumetricServer::streamVisibleCells(const ViewportData& viewport, const struct sockaddr_in& clientAddr) {
    auto visibleCells = cellManager->getVisibleCells(viewport);
    
    std::cout << "[VolumetricServer::streamVisibleCells] Found " << visibleCells.size() 
              << " visible cells to stream to client at "
              << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << std::endl;
    
    if (visibleCells.empty()) {
        std::cout << "[VolumetricServer::streamVisibleCells] No visible cells to stream" << std::endl;
        return;
    }
    
    // Stream cells in order of importance
    for (const auto* cell : visibleCells) {
        sendCell(*cell, clientAddr);
    }
}

void VolumetricServer::sendCell(const Cell& cell, const struct sockaddr_in& clientAddr) {
    std::cout << "[VolumetricServer::sendCell] Starting to send cell to "
              << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port)
              << " with " << cell.points->size() << " points" << std::endl;
    
    // Compress cell data
    std::vector<char> compressedData;
    try {
        compressor->compress(cell.points, compressedData);
        std::cout << "[VolumetricServer::sendCell] Successfully compressed data from "
                  << (cell.points->size() * sizeof(pcl::PointXYZRGB)) << " bytes to "
                  << compressedData.size() << " bytes" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[VolumetricServer::sendCell] Compression failed: " << e.what() << std::endl;
        return;
    }
    
    // Split compressed data into chunks
    size_t totalChunks = (compressedData.size() + MAX_PACKET_SIZE - CHUNK_HEADER_SIZE - 1) / 
                        (MAX_PACKET_SIZE - CHUNK_HEADER_SIZE);
    
    std::cout << "[VolumetricServer::sendCell] Splitting " << compressedData.size() 
              << " bytes into " << totalChunks << " chunks" << std::endl;
    
    static uint32_t cellIdCounter = 0;
    uint32_t cellId = cellIdCounter++;
    
    size_t totalBytesSent = 0;
    std::vector<char> packet(MAX_PACKET_SIZE);
    for (size_t i = 0; i < totalChunks; ++i) {
        // Prepare packet header
        PacketHeader* header = reinterpret_cast<PacketHeader*>(packet.data());
        header->type = PacketType::POINT_CLOUD_DATA;
        header->sequenceNumber = i;
        header->timestamp = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
        header->totalChunks = totalChunks;
        header->chunkIndex = i;
        
        // Calculate chunk size
        size_t chunkStart = i * (MAX_PACKET_SIZE - sizeof(PacketHeader));
        size_t chunkSize = std::min(static_cast<size_t>(MAX_PACKET_SIZE - sizeof(PacketHeader)),
                                  compressedData.size() - chunkStart);
        header->payloadSize = chunkSize;
        
        // Copy chunk data
        std::memcpy(packet.data() + sizeof(PacketHeader), 
                   &compressedData[chunkStart], chunkSize);
        
        size_t totalPacketSize = sizeof(PacketHeader) + chunkSize;
        
        // Send to client
        std::cout << "[VolumetricServer::sendCell] Sending chunk " << (i + 1) << "/" << totalChunks 
                  << " (size: " << totalPacketSize << " bytes) to "
                  << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << std::endl;
                  
        ssize_t bytesSent = sendto(sockfd, packet.data(), totalPacketSize, 0,
                                  (struct sockaddr*)&clientAddr, sizeof(clientAddr));
                                  
        if (bytesSent < 0) {
            std::cerr << "[VolumetricServer::sendCell] Failed to send chunk " << (i + 1) 
                      << ": " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        } else {
            totalBytesSent += bytesSent;
            std::cout << "[VolumetricServer::sendCell] Successfully sent chunk " << (i + 1) << "/" 
                      << totalChunks << " (" << bytesSent << " bytes)" << std::endl;
        }
        
        // Small delay to prevent network congestion
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    std::cout << "[VolumetricServer::sendCell] Finished sending cell " << cellId 
              << " to client. Total bytes sent: " << totalBytesSent << std::endl;
}

std::string VolumetricServer::getClientKey(const struct sockaddr_in& addr) const {
    char ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);
    return std::string(ip) + ":" + std::to_string(ntohs(addr.sin_port));
}

void VolumetricServer::predictViewport(ViewportData& viewport) const {
    // Simple linear prediction based on previous position
    // In a real implementation, this would use more sophisticated ML models
    static const float PREDICTION_WINDOW = 0.1f; // 100ms prediction
    
    // For now, just keep the current viewport
    // TODO: Implement ML-based prediction
}

void VolumetricServer::setupSocket() {
    // Close any existing socket
    if (sockfd >= 0) {
        std::cout << "[VolumetricServer::setupSocket] Closing existing socket fd: " << sockfd << std::endl;
        close(sockfd);
        sockfd = -1;
    }

    // Create new socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "[VolumetricServer::setupSocket] Error creating socket: " << strerror(errno) << std::endl;
        exit(1);
    }
    std::cout << "[VolumetricServer::setupSocket] Created socket with fd: " << sockfd << std::endl;

    // Enable socket reuse
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "[VolumetricServer::setupSocket] Warning: Could not set SO_REUSEADDR: " 
                  << strerror(errno) << std::endl;
    } else {
        std::cout << "[VolumetricServer::setupSocket] Set SO_REUSEADDR" << std::endl;
    }

    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "[VolumetricServer::setupSocket] Warning: Could not set SO_REUSEPORT: " 
                  << strerror(errno) << std::endl;
    } else {
        std::cout << "[VolumetricServer::setupSocket] Set SO_REUSEPORT" << std::endl;
    }

    // Enable socket buffer size option
    int sendbuff = 1024 * 1024; // 1MB buffer
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sendbuff, sizeof(sendbuff)) < 0) {
        std::cerr << "[VolumetricServer::setupSocket] Warning: Could not set send buffer size: " 
                  << strerror(errno) << std::endl;
    } else {
        std::cout << "[VolumetricServer::setupSocket] Set send buffer size to 1MB" << std::endl;
    }

    // Set receive buffer size
    int rcvbuf = 1024 * 1024; // 1MB buffer
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
        std::cerr << "[VolumetricServer::setupSocket] Warning: Could not set receive buffer size: " 
                  << strerror(errno) << std::endl;
    } else {
        std::cout << "[VolumetricServer::setupSocket] Set receive buffer size to 1MB" << std::endl;
    }

    // Set non-blocking mode
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) {
        std::cerr << "[VolumetricServer::setupSocket] Warning: Could not get socket flags: " 
                  << strerror(errno) << std::endl;
    } else {
        if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
            std::cerr << "[VolumetricServer::setupSocket] Warning: Could not set non-blocking mode: " 
                      << strerror(errno) << std::endl;
        } else {
            std::cout << "[VolumetricServer::setupSocket] Set socket to non-blocking mode" << std::endl;
        }
    }

    // Get the actual buffer sizes
    int actualSendBuf = 0;
    int actualRcvBuf = 0;
    socklen_t optlen = sizeof(int);
    
    if (getsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &actualSendBuf, &optlen) == 0) {
        std::cout << "[VolumetricServer::setupSocket] Actual send buffer size: " 
                  << actualSendBuf << " bytes" << std::endl;
    }
    
    if (getsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &actualRcvBuf, &optlen) == 0) {
        std::cout << "[VolumetricServer::setupSocket] Actual receive buffer size: " 
                  << actualRcvBuf << " bytes" << std::endl;
    }

    // Set up server address
    memset(&serverAddr, 0, sizeof(serverAddr));  // Clear the structure first
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(PORT);

    std::cout << "[VolumetricServer::setupSocket] Binding to port " << PORT 
              << " on all interfaces..." << std::endl;
    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "[VolumetricServer::setupSocket] Error binding socket: " 
                  << strerror(errno) << std::endl;
        exit(1);
    }
    
    // Get the actual bound address and port
    struct sockaddr_in boundAddr;
    socklen_t boundAddrLen = sizeof(boundAddr);
    if (getsockname(sockfd, (struct sockaddr*)&boundAddr, &boundAddrLen) == 0) {
        char boundIP[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &boundAddr.sin_addr, boundIP, INET_ADDRSTRLEN);
        std::cout << "[VolumetricServer::setupSocket] Successfully bound to " 
                  << boundIP << ":" << ntohs(boundAddr.sin_port) << std::endl;
    } else {
        std::cerr << "[VolumetricServer::setupSocket] Warning: Could not get bound address: " 
                  << strerror(errno) << std::endl;
    }
}

void VolumetricServer::generateSamplePointCloud() {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // TODO: Load actual point cloud data
    // For now, just create a simple cube of points
    for (float x = -1; x <= 1; x += 0.1) {
        for (float y = -1; y <= 1; y += 0.1) {
            for (float z = -1; z <= 1; z += 0.1) {
                pcl::PointXYZRGB point;
                point.x = x * 100; // Scale to 100cm
                point.y = y * 100;
                point.z = z * 100;
                point.r = static_cast<uint8_t>((x + 1) * 127.5);
                point.g = static_cast<uint8_t>((y + 1) * 127.5);
                point.b = static_cast<uint8_t>((z + 1) * 127.5);
                cloud->push_back(point);
            }
        }
    }
}

} // namespace hologram 