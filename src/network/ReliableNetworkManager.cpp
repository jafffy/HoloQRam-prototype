#include "hologram/network/ReliableNetworkManager.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

namespace hologram {

ReliableNetworkManager::ReliableNetworkManager(int sockfd)
    : sockfd(sockfd)
    , running(false)
    , currentSequence(0)
    , expectedSequence(0)
{
}

ReliableNetworkManager::~ReliableNetworkManager() {
    stop();
}

void ReliableNetworkManager::start() {
    if (running) return;
    
    running = true;
    
    // Start worker threads
    senderThread = std::make_unique<std::thread>(&ReliableNetworkManager::senderLoop, this);
    receiverThread = std::make_unique<std::thread>(&ReliableNetworkManager::receiverLoop, this);
    retransmitThread = std::make_unique<std::thread>(&ReliableNetworkManager::retransmitLoop, this);
    cleanupThread = std::make_unique<std::thread>(&ReliableNetworkManager::cleanupLoop, this);
    adaptiveControlThread = std::make_unique<std::thread>(&ReliableNetworkManager::adaptiveControlLoop, this);
}

void ReliableNetworkManager::stop() {
    if (!running) return;
    
    running = false;
    sendQueueCV.notify_all();
    
    // Join all threads
    if (senderThread && senderThread->joinable()) senderThread->join();
    if (receiverThread && receiverThread->joinable()) receiverThread->join();
    if (retransmitThread && retransmitThread->joinable()) retransmitThread->join();
    if (cleanupThread && cleanupThread->joinable()) cleanupThread->join();
    if (adaptiveControlThread && adaptiveControlThread->joinable()) adaptiveControlThread->join();
    
    // Clear queues and maps
    {
        std::lock_guard<std::mutex> lock(packetsMutex);
        sentPackets.clear();
        receivedChunks.clear();
    }
    {
        std::lock_guard<std::mutex> lock(sendQueueMutex);
        std::queue<std::vector<char>>().swap(sendQueue);
    }
}

bool ReliableNetworkManager::sendReliable(const std::vector<char>& data, PacketType type) {
    if (!running) return false;
    
    // Split data into chunks if needed
    auto chunks = chunkData(data, type);
    
    // Queue chunks for sending
    {
        std::lock_guard<std::mutex> lock(sendQueueMutex);
        for (const auto& chunk : chunks) {
            sendQueue.push(chunk);
        }
    }
    sendQueueCV.notify_one();
    
    return true;
}

void ReliableNetworkManager::setDataCallback(DataCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex);
    dataCallback = callback;
}

PacketMetrics ReliableNetworkManager::getPacketMetrics() const {
    return metrics.getPacketMetrics();
}

CompressionMetrics ReliableNetworkManager::getCompressionMetrics() const {
    return metrics.getCompressionMetrics();
}

AdaptiveNetworkControl::NetworkParams ReliableNetworkManager::getNetworkParams() const {
    return adaptiveControl.getCurrentParams();
}

void ReliableNetworkManager::senderLoop() {
    while (running) {
        std::vector<char> data;
        {
            std::unique_lock<std::mutex> lock(sendQueueMutex);
            sendQueueCV.wait(lock, [this]() {
                return !sendQueue.empty() || !running;
            });
            
            if (!running) break;
            
            data = std::move(sendQueue.front());
            sendQueue.pop();
        }
        
        // Send packet
        if (send(sockfd, data.data(), data.size(), 0) < 0) {
            std::cerr << "Error sending packet" << std::endl;
            continue;
        }
        
        // Record metrics
        metrics.recordPacketSent(data.size());
        
        // Store packet for potential retransmission
        PacketHeader* header = reinterpret_cast<PacketHeader*>(data.data());
        {
            std::lock_guard<std::mutex> lock(packetsMutex);
            PacketInfo& info = sentPackets[header->sequenceNumber];
            info.sendTime = std::chrono::steady_clock::now();
            info.retryCount = 0;
            info.acknowledged = false;
            info.data = std::move(data);
        }
        
        // Apply rate control
        auto delay = calculatePacketDelay();
        if (delay.count() > 0) {
            std::this_thread::sleep_for(delay);
        }
    }
}

void ReliableNetworkManager::receiverLoop() {
    char buffer[MAX_PACKET_SIZE];
    
    while (running) {
        ssize_t bytesReceived = recv(sockfd, buffer, sizeof(buffer), 0);
        if (bytesReceived <= 0) continue;
        
        handleIncomingPacket(buffer, bytesReceived);
    }
}

void ReliableNetworkManager::retransmitLoop() {
    while (running) {
        {
            std::lock_guard<std::mutex> lock(packetsMutex);
            auto now = std::chrono::steady_clock::now();
            
            for (auto& [seq, info] : sentPackets) {
                if (info.acknowledged) continue;
                
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - info.sendTime
                ).count();
                
                if (elapsed >= ACK_TIMEOUT_MS && info.retryCount < MAX_RETRIES) {
                    // Retransmit packet
                    if (send(sockfd, info.data.data(), info.data.size(), 0) < 0) {
                        std::cerr << "Error retransmitting packet" << std::endl;
                        continue;
                    }
                    
                    info.sendTime = now;
                    info.retryCount++;
                    metrics.recordRetransmission();
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(ACK_TIMEOUT_MS / 2));
    }
}

void ReliableNetworkManager::cleanupLoop() {
    while (running) {
        cleanupOldPackets();
        std::this_thread::sleep_for(std::chrono::milliseconds(CLEANUP_INTERVAL_MS));
    }
}

void ReliableNetworkManager::adaptiveControlLoop() {
    while (running) {
        updateMetrics();
        adaptiveControl.updateParameters(metrics.getPacketMetrics(), metrics.getCompressionMetrics());
        std::this_thread::sleep_for(ADAPTIVE_CONTROL_INTERVAL);
    }
}

void ReliableNetworkManager::handleIncomingPacket(const char* data, size_t length) {
    if (length < sizeof(PacketHeader)) return;
    
    const PacketHeader* header = reinterpret_cast<const PacketHeader*>(data);
    
    switch (header->type) {
        case PacketType::ACK:
            if (length >= sizeof(AckPacket)) {
                handleAck(reinterpret_cast<const AckPacket*>(data));
            }
            break;
            
        case PacketType::RETRANSMIT_REQUEST:
            if (length >= sizeof(RetransmitRequest)) {
                handleRetransmitRequest(reinterpret_cast<const RetransmitRequest*>(data));
            }
            break;
            
        default:
            // Regular data packet
            metrics.recordPacketReceived(length);
            
            // Send ACK
            sendAck(header->sequenceNumber);
            
            // Store chunk
            {
                std::lock_guard<std::mutex> lock(packetsMutex);
                auto& chunks = receivedChunks[header->sequenceNumber];
                if (chunks.size() <= header->chunkIndex) {
                    chunks.resize(header->totalChunks);
                }
                chunks[header->chunkIndex].assign(data, data + length);
                
                // Try to reassemble complete message
                if (tryReassembleMessage(header->sequenceNumber)) {
                    receivedChunks.erase(header->sequenceNumber);
                }
            }
            break;
    }
}

void ReliableNetworkManager::handleAck(const AckPacket* ack) {
    std::lock_guard<std::mutex> lock(packetsMutex);
    auto it = sentPackets.find(ack->sequenceNumber);
    if (it != sentPackets.end()) {
        it->second.acknowledged = true;
        recordRTT(ack->sequenceNumber);
    }
}

void ReliableNetworkManager::handleRetransmitRequest(const RetransmitRequest* req) {
    std::lock_guard<std::mutex> lock(packetsMutex);
    auto it = sentPackets.find(req->sequenceNumber);
    if (it != sentPackets.end() && !it->second.acknowledged) {
        retransmitPacket(req->sequenceNumber, req->chunkIndex);
    }
}

void ReliableNetworkManager::sendAck(uint32_t sequenceNumber) {
    AckPacket ack;
    ack.type = PacketType::ACK;
    ack.sequenceNumber = sequenceNumber;
    ack.timestamp = getCurrentTimestamp();
    
    if (send(sockfd, &ack, sizeof(ack), 0) < 0) {
        std::cerr << "Error sending ACK" << std::endl;
    }
}

void ReliableNetworkManager::retransmitPacket(uint32_t sequenceNumber, uint32_t chunkIndex) {
    auto it = sentPackets.find(sequenceNumber);
    if (it == sentPackets.end()) return;
    
    const auto& packet = it->second.data;
    if (send(sockfd, packet.data(), packet.size(), 0) < 0) {
        std::cerr << "Error retransmitting packet" << std::endl;
        return;
    }
    
    it->second.sendTime = std::chrono::steady_clock::now();
    it->second.retryCount++;
    metrics.recordRetransmission();
}

void ReliableNetworkManager::cleanupOldPackets() {
    std::lock_guard<std::mutex> lock(packetsMutex);
    auto now = std::chrono::steady_clock::now();
    
    // Remove acknowledged packets
    for (auto it = sentPackets.begin(); it != sentPackets.end();) {
        if (it->second.acknowledged) {
            it = sentPackets.erase(it);
        } else {
            ++it;
        }
    }
    
    // Remove old unacknowledged packets that exceeded max retries
    for (auto it = sentPackets.begin(); it != sentPackets.end();) {
        if (it->second.retryCount >= MAX_RETRIES) {
            metrics.recordPacketLost();
            it = sentPackets.erase(it);
        } else {
            ++it;
        }
    }
    
    // Remove old incomplete messages
    for (auto it = receivedChunks.begin(); it != receivedChunks.end();) {
        bool incomplete = false;
        for (const auto& chunk : it->second) {
            if (chunk.empty()) {
                incomplete = true;
                break;
            }
        }
        if (incomplete) {
            it = receivedChunks.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<std::vector<char>> ReliableNetworkManager::chunkData(
    const std::vector<char>& data, PacketType type) {
    
    std::vector<std::vector<char>> chunks;
    size_t maxChunkSize = MAX_PAYLOAD_SIZE - sizeof(PacketHeader);
    size_t totalChunks = (data.size() + maxChunkSize - 1) / maxChunkSize;
    
    for (size_t i = 0; i < totalChunks; ++i) {
        size_t offset = i * maxChunkSize;
        size_t size = std::min(maxChunkSize, data.size() - offset);
        
        std::vector<char> chunk(sizeof(PacketHeader) + size);
        PacketHeader* header = reinterpret_cast<PacketHeader*>(chunk.data());
        header->type = type;
        header->sequenceNumber = getAndIncrementSequence();
        header->timestamp = getCurrentTimestamp();
        header->totalChunks = totalChunks;
        header->chunkIndex = i;
        header->payloadSize = size;
        
        std::memcpy(chunk.data() + sizeof(PacketHeader),
                   data.data() + offset,
                   size);
        
        chunks.push_back(std::move(chunk));
    }
    
    return chunks;
}

bool ReliableNetworkManager::tryReassembleMessage(uint32_t sequenceNumber) {
    auto it = receivedChunks.find(sequenceNumber);
    if (it == receivedChunks.end()) return false;
    
    const auto& chunks = it->second;
    
    // Check if all chunks are received
    for (const auto& chunk : chunks) {
        if (chunk.empty()) return false;
    }
    
    // Reassemble message
    std::vector<char> message;
    size_t totalSize = 0;
    for (const auto& chunk : chunks) {
        totalSize += chunk.size() - sizeof(PacketHeader);
    }
    message.reserve(totalSize);
    
    for (const auto& chunk : chunks) {
        const PacketHeader* header = reinterpret_cast<const PacketHeader*>(chunk.data());
        message.insert(message.end(),
                      chunk.data() + sizeof(PacketHeader),
                      chunk.data() + chunk.size());
    }
    
    // Notify callback
    if (dataCallback) {
        const PacketHeader* firstHeader = reinterpret_cast<const PacketHeader*>(chunks[0].data());
        dataCallback(message, firstHeader->type);
    }
    
    return true;
}

uint32_t ReliableNetworkManager::getAndIncrementSequence() {
    std::lock_guard<std::mutex> lock(sequenceMutex);
    uint32_t seq = currentSequence;
    currentSequence = getNextSequenceNumber(currentSequence);
    return seq;
}

uint32_t ReliableNetworkManager::getCurrentTimestamp() {
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
}

void ReliableNetworkManager::updateMetrics() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastMetricsUpdate
    );
    
    if (elapsed >= METRICS_UPDATE_INTERVAL) {
        lastMetricsUpdate = now;
        applyRateControl();
    }
}

void ReliableNetworkManager::recordPacketMetrics(size_t bytes, bool sent) {
    if (sent) {
        metrics.recordPacketSent(bytes);
    } else {
        metrics.recordPacketReceived(bytes);
    }
}

void ReliableNetworkManager::recordRTT(uint32_t sequenceNumber) {
    auto it = sentPackets.find(sequenceNumber);
    if (it != sentPackets.end()) {
        auto now = std::chrono::steady_clock::now();
        float rtt = std::chrono::duration<float, std::milli>(
            now - it->second.sendTime
        ).count();
        metrics.recordRoundTripTime(rtt);
    }
}

void ReliableNetworkManager::applyRateControl() {
    auto params = adaptiveControl.getCurrentParams();
    // Rate control logic is applied in calculatePacketDelay()
}

bool ReliableNetworkManager::shouldSendPacket() const {
    auto params = adaptiveControl.getCurrentParams();
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(packetsMutex));
    return sentPackets.size() < static_cast<size_t>(params.congestionWindow);
}

std::chrono::microseconds ReliableNetworkManager::calculatePacketDelay() const {
    auto params = adaptiveControl.getCurrentParams();
    if (params.transmissionRate <= 0) return std::chrono::microseconds(0);
    
    float bytesPerMicrosecond = params.transmissionRate / 1'000'000.0f;
    float microsecondsPerByte = 1.0f / bytesPerMicrosecond;
    
    return std::chrono::microseconds(
        static_cast<long long>(microsecondsPerByte * MAX_PACKET_SIZE)
    );
} 

} // namespace hologram 