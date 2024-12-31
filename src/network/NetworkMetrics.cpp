#include "hologram/network/NetworkMetrics.hpp"
#include <algorithm>
#include <numeric>

NetworkMetrics::NetworkMetrics() {
    // Initialize atomic counters to 0 (done in header)
}

void NetworkMetrics::recordPacketSent(size_t bytes) {
    totalBytesSent += bytes;
    packetsSent++;
    
    // Record bandwidth sample
    std::lock_guard<std::mutex> lock(bwMutex);
    bandwidthSamples.push_back({bytes, std::chrono::steady_clock::now()});
    cleanupOldSamples();
}

void NetworkMetrics::recordPacketReceived(size_t bytes) {
    totalBytesReceived += bytes;
    packetsReceived++;
}

void NetworkMetrics::recordPacketLost() {
    packetsLost++;
}

void NetworkMetrics::recordRetransmission() {
    packetsRetransmitted++;
}

void NetworkMetrics::recordRoundTripTime(float rttMs) {
    std::lock_guard<std::mutex> lock(rttMutex);
    rttSamples.push_back({rttMs, std::chrono::steady_clock::now()});
    cleanupOldSamples();
}

void NetworkMetrics::recordCompression(size_t originalSize, size_t compressedSize, float compressionTimeMs) {
    std::lock_guard<std::mutex> lock(compressionMutex);
    compressionSamples.push_back({
        originalSize,
        compressedSize,
        compressionTimeMs,
        std::chrono::steady_clock::now()
    });
    cleanupOldSamples();
}

void NetworkMetrics::recordDecompression(size_t compressedSize, size_t decompressedSize, float decompressionTimeMs) {
    std::lock_guard<std::mutex> lock(compressionMutex);
    decompressionSamples.push_back({
        compressedSize,
        decompressedSize,
        decompressionTimeMs,
        std::chrono::steady_clock::now()
    });
    cleanupOldSamples();
}

PacketMetrics NetworkMetrics::getPacketMetrics() const {
    PacketMetrics metrics;
    metrics.totalBytesSent = totalBytesSent;
    metrics.totalBytesReceived = totalBytesReceived;
    metrics.packetsSent = packetsSent;
    metrics.packetsReceived = packetsReceived;
    metrics.packetsLost = packetsLost;
    metrics.packetsRetransmitted = packetsRetransmitted;
    metrics.averageRoundTripTime = calculateAverageRTT();
    metrics.packetLossRate = calculatePacketLossRate();
    metrics.currentBandwidth = calculateCurrentBandwidth();
    return metrics;
}

CompressionMetrics NetworkMetrics::getCompressionMetrics() const {
    CompressionMetrics metrics;
    
    std::lock_guard<std::mutex> lock(compressionMutex);
    
    metrics.totalUncompressedBytes = 0;
    metrics.totalCompressedBytes = 0;
    
    for (const auto& sample : compressionSamples) {
        metrics.totalUncompressedBytes += sample.originalSize;
        metrics.totalCompressedBytes += sample.compressedSize;
    }
    
    metrics.averageCompressionRatio = calculateAverageCompressionRatio();
    metrics.averageCompressionTime = calculateAverageCompressionTime();
    metrics.averageDecompressionTime = calculateAverageDecompressionTime();
    
    return metrics;
}

void NetworkMetrics::cleanupOldSamples() {
    auto now = std::chrono::steady_clock::now();
    auto windowDuration = std::chrono::milliseconds(METRICS_WINDOW_MS);
    
    {
        std::lock_guard<std::mutex> lock(rttMutex);
        while (!rttSamples.empty() && 
               now - rttSamples.front().timestamp > windowDuration) {
            rttSamples.pop_front();
        }
    }
    
    {
        std::lock_guard<std::mutex> lock(bwMutex);
        while (!bandwidthSamples.empty() && 
               now - bandwidthSamples.front().timestamp > windowDuration) {
            bandwidthSamples.pop_front();
        }
    }
    
    {
        std::lock_guard<std::mutex> lock(compressionMutex);
        while (!compressionSamples.empty() && 
               now - compressionSamples.front().timestamp > windowDuration) {
            compressionSamples.pop_front();
        }
        while (!decompressionSamples.empty() && 
               now - decompressionSamples.front().timestamp > windowDuration) {
            decompressionSamples.pop_front();
        }
    }
}

float NetworkMetrics::calculateAverageRTT() const {
    std::lock_guard<std::mutex> lock(rttMutex);
    if (rttSamples.empty()) return 0.0f;
    
    float sum = std::accumulate(rttSamples.begin(), rttSamples.end(), 0.0f,
        [](float acc, const RTTSample& sample) {
            return acc + sample.rttMs;
        });
    
    return sum / rttSamples.size();
}

float NetworkMetrics::calculatePacketLossRate() const {
    uint64_t totalPackets = packetsSent.load();
    if (totalPackets == 0) return 0.0f;
    
    return (static_cast<float>(packetsLost.load()) / totalPackets) * 100.0f;
}

float NetworkMetrics::calculateCurrentBandwidth() const {
    std::lock_guard<std::mutex> lock(bwMutex);
    if (bandwidthSamples.empty()) return 0.0f;
    
    auto now = std::chrono::steady_clock::now();
    auto oldestTime = bandwidthSamples.front().timestamp;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - oldestTime).count() / 1000.0f;
    
    if (duration <= 0.0f) return 0.0f;
    
    uint64_t totalBytes = std::accumulate(bandwidthSamples.begin(), bandwidthSamples.end(), 0ULL,
        [](uint64_t acc, const BandwidthSample& sample) {
            return acc + sample.bytes;
        });
    
    return static_cast<float>(totalBytes) / duration;
}

float NetworkMetrics::calculateAverageCompressionRatio() const {
    std::lock_guard<std::mutex> lock(compressionMutex);
    if (compressionSamples.empty()) return 1.0f;
    
    float sum = 0.0f;
    for (const auto& sample : compressionSamples) {
        if (sample.originalSize > 0) {
            sum += static_cast<float>(sample.compressedSize) / sample.originalSize;
        }
    }
    
    return sum / compressionSamples.size();
}

float NetworkMetrics::calculateAverageCompressionTime() const {
    std::lock_guard<std::mutex> lock(compressionMutex);
    if (compressionSamples.empty()) return 0.0f;
    
    float sum = std::accumulate(compressionSamples.begin(), compressionSamples.end(), 0.0f,
        [](float acc, const CompressionSample& sample) {
            return acc + sample.processingTimeMs;
        });
    
    return sum / compressionSamples.size();
}

float NetworkMetrics::calculateAverageDecompressionTime() const {
    std::lock_guard<std::mutex> lock(compressionMutex);
    if (decompressionSamples.empty()) return 0.0f;
    
    float sum = std::accumulate(decompressionSamples.begin(), decompressionSamples.end(), 0.0f,
        [](float acc, const CompressionSample& sample) {
            return acc + sample.processingTimeMs;
        });
    
    return sum / decompressionSamples.size();
} 