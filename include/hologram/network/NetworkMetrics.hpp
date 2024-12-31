#pragma once

#include <cstdint>
#include <chrono>
#include <deque>
#include <mutex>
#include <atomic>

// Time window for metrics (5 seconds)
constexpr size_t METRICS_WINDOW_MS = 5000;

struct PacketMetrics {
    uint64_t totalBytesSent;
    uint64_t totalBytesReceived;
    uint64_t packetsSent;
    uint64_t packetsReceived;
    uint64_t packetsLost;
    uint64_t packetsRetransmitted;
    float averageRoundTripTime;  // milliseconds
    float packetLossRate;        // percentage
    float currentBandwidth;      // bytes per second
};

struct CompressionMetrics {
    uint64_t totalUncompressedBytes;
    uint64_t totalCompressedBytes;
    float averageCompressionRatio;
    float averageCompressionTime;  // milliseconds
    float averageDecompressionTime; // milliseconds
};

class NetworkMetrics {
public:
    NetworkMetrics();

    // Record events
    void recordPacketSent(size_t bytes);
    void recordPacketReceived(size_t bytes);
    void recordPacketLost();
    void recordRetransmission();
    void recordRoundTripTime(float rttMs);
    void recordCompression(size_t originalSize, size_t compressedSize, float compressionTimeMs);
    void recordDecompression(size_t compressedSize, size_t decompressedSize, float decompressionTimeMs);

    // Get metrics
    PacketMetrics getPacketMetrics() const;
    CompressionMetrics getCompressionMetrics() const;

private:
    // Packet metrics
    std::atomic<uint64_t> totalBytesSent{0};
    std::atomic<uint64_t> totalBytesReceived{0};
    std::atomic<uint64_t> packetsSent{0};
    std::atomic<uint64_t> packetsReceived{0};
    std::atomic<uint64_t> packetsLost{0};
    std::atomic<uint64_t> packetsRetransmitted{0};

    // RTT tracking
    struct RTTSample {
        float rttMs;
        std::chrono::steady_clock::time_point timestamp;
    };
    mutable std::mutex rttMutex;
    std::deque<RTTSample> rttSamples;

    // Bandwidth tracking
    struct BandwidthSample {
        uint64_t bytes;
        std::chrono::steady_clock::time_point timestamp;
    };
    mutable std::mutex bwMutex;
    std::deque<BandwidthSample> bandwidthSamples;

    // Compression metrics
    struct CompressionSample {
        size_t originalSize;
        size_t compressedSize;
        float processingTimeMs;
        std::chrono::steady_clock::time_point timestamp;
    };
    mutable std::mutex compressionMutex;
    std::deque<CompressionSample> compressionSamples;
    std::deque<CompressionSample> decompressionSamples;

    // Helper functions
    void cleanupOldSamples();
    float calculateAverageRTT() const;
    float calculatePacketLossRate() const;
    float calculateCurrentBandwidth() const;
    float calculateAverageCompressionRatio() const;
    float calculateAverageCompressionTime() const;
    float calculateAverageDecompressionTime() const;
}; 