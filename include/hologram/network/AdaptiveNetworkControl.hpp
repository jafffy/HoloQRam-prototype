#pragma once

#include "hologram/network/NetworkMetrics.hpp"
#include <chrono>
#include <mutex>

class AdaptiveNetworkControl {
public:
    struct NetworkParams {
        size_t chunkSize;           // Current optimal chunk size
        float transmissionRate;      // Bytes per second
        uint32_t retryTimeout;      // Milliseconds before retry
        uint32_t maxRetries;        // Maximum retry attempts
        float congestionWindow;      // Number of packets in flight
    };

    AdaptiveNetworkControl();

    // Update parameters based on current metrics
    void updateParameters(const PacketMetrics& metrics, const CompressionMetrics& compression);

    // Get current parameters
    NetworkParams getCurrentParams() const;

    // Reset to default parameters
    void reset();

private:
    static constexpr size_t MIN_CHUNK_SIZE = 1024;      // 1KB
    static constexpr size_t MAX_CHUNK_SIZE = 63 * 1024; // 63KB (leaving room for headers)
    static constexpr float MIN_TRANSMISSION_RATE = 10 * 1024;    // 10KB/s
    static constexpr float MAX_TRANSMISSION_RATE = 100 * 1024 * 1024; // 100MB/s
    static constexpr uint32_t MIN_RETRY_TIMEOUT = 20;    // 20ms
    static constexpr uint32_t MAX_RETRY_TIMEOUT = 1000;  // 1s
    static constexpr float MIN_CONGESTION_WINDOW = 1.0f;
    static constexpr float MAX_CONGESTION_WINDOW = 64.0f;

    // Current parameters
    NetworkParams params;
    mutable std::mutex paramsMutex;

    // State for congestion control
    enum class CongestionState {
        SLOW_START,
        CONGESTION_AVOIDANCE,
        FAST_RECOVERY
    };
    CongestionState congestionState;

    // Metrics history for trend analysis
    struct MetricsSnapshot {
        float rtt;
        float packetLoss;
        float bandwidth;
        std::chrono::steady_clock::time_point timestamp;
    };
    static constexpr size_t HISTORY_SIZE = 10;
    std::array<MetricsSnapshot, HISTORY_SIZE> metricsHistory;
    size_t historyIndex;

    // Helper functions
    void adjustChunkSize(float rtt, float packetLoss, float bandwidth);
    void adjustTransmissionRate(float rtt, float packetLoss, float bandwidth);
    void adjustRetryTimeout(float rtt);
    void adjustCongestionWindow(float packetLoss);
    void updateMetricsHistory(float rtt, float packetLoss, float bandwidth);
    bool detectNetworkDegradation() const;
    
    // Utility functions
    template<typename T>
    T clamp(T value, T min, T max) const {
        return std::min(std::max(value, min), max);
    }
}; 