#include "hologram/network/AdaptiveNetworkControl.hpp"
#include <algorithm>
#include <cmath>

AdaptiveNetworkControl::AdaptiveNetworkControl()
    : congestionState(CongestionState::SLOW_START)
    , historyIndex(0)
{
    reset();
}

void AdaptiveNetworkControl::updateParameters(
    const PacketMetrics& metrics,
    const CompressionMetrics& compression)
{
    std::lock_guard<std::mutex> lock(paramsMutex);
    
    // Update metrics history
    updateMetricsHistory(
        metrics.averageRoundTripTime,
        metrics.packetLossRate,
        metrics.currentBandwidth
    );
    
    // Adjust parameters based on network conditions
    adjustChunkSize(
        metrics.averageRoundTripTime,
        metrics.packetLossRate,
        metrics.currentBandwidth
    );
    
    adjustTransmissionRate(
        metrics.averageRoundTripTime,
        metrics.packetLossRate,
        metrics.currentBandwidth
    );
    
    adjustRetryTimeout(metrics.averageRoundTripTime);
    adjustCongestionWindow(metrics.packetLossRate);
}

AdaptiveNetworkControl::NetworkParams AdaptiveNetworkControl::getCurrentParams() const {
    std::lock_guard<std::mutex> lock(paramsMutex);
    return params;
}

void AdaptiveNetworkControl::reset() {
    std::lock_guard<std::mutex> lock(paramsMutex);
    
    params.chunkSize = 8 * 1024;  // Start with 8KB chunks
    params.transmissionRate = 1024 * 1024;  // Start with 1MB/s
    params.retryTimeout = 100;  // Start with 100ms timeout
    params.maxRetries = 3;
    params.congestionWindow = 1.0f;  // Start with 1 packet in flight
    
    congestionState = CongestionState::SLOW_START;
    historyIndex = 0;
}

void AdaptiveNetworkControl::adjustChunkSize(
    float rtt, float packetLoss, float bandwidth)
{
    // Adjust chunk size based on network conditions
    float targetChunkSize = params.chunkSize;
    
    if (packetLoss < 1.0f && rtt < 50.0f) {
        // Good network conditions - try larger chunks
        targetChunkSize *= 1.1f;
    } else if (packetLoss > 5.0f || rtt > 200.0f) {
        // Poor network conditions - use smaller chunks
        targetChunkSize *= 0.9f;
    }
    
    // Consider bandwidth
    float maxChunkSize = bandwidth * 0.1f;  // Max 100ms worth of data
    targetChunkSize = std::min(targetChunkSize, maxChunkSize);
    
    params.chunkSize = clamp(
        static_cast<size_t>(targetChunkSize),
        MIN_CHUNK_SIZE,
        MAX_CHUNK_SIZE
    );
}

void AdaptiveNetworkControl::adjustTransmissionRate(
    float rtt, float packetLoss, float bandwidth)
{
    float targetRate = params.transmissionRate;
    
    switch (congestionState) {
        case CongestionState::SLOW_START:
            if (packetLoss > 1.0f || rtt > 200.0f) {
                congestionState = CongestionState::CONGESTION_AVOIDANCE;
                targetRate *= 0.5f;
            } else {
                targetRate *= 2.0f;
            }
            break;
            
        case CongestionState::CONGESTION_AVOIDANCE:
            if (packetLoss < 0.1f && rtt < 100.0f) {
                targetRate *= 1.1f;
            } else if (packetLoss > 5.0f || rtt > 300.0f) {
                targetRate *= 0.5f;
                congestionState = CongestionState::FAST_RECOVERY;
            } else {
                targetRate *= 1.01f;
            }
            break;
            
        case CongestionState::FAST_RECOVERY:
            if (packetLoss < 1.0f && rtt < 200.0f) {
                congestionState = CongestionState::CONGESTION_AVOIDANCE;
                targetRate *= 1.05f;
            } else {
                targetRate *= 0.95f;
            }
            break;
    }
    
    params.transmissionRate = clamp(
        targetRate,
        MIN_TRANSMISSION_RATE,
        MAX_TRANSMISSION_RATE
    );
}

void AdaptiveNetworkControl::adjustRetryTimeout(float rtt) {
    // Set timeout to RTT + margin
    float targetTimeout = rtt * 2.0f;  // 2x RTT
    
    // Add jitter margin
    targetTimeout += 20.0f;  // Add 20ms margin
    
    params.retryTimeout = clamp(
        static_cast<uint32_t>(targetTimeout),
        MIN_RETRY_TIMEOUT,
        MAX_RETRY_TIMEOUT
    );
}

void AdaptiveNetworkControl::adjustCongestionWindow(float packetLoss) {
    float targetWindow = params.congestionWindow;
    
    if (packetLoss < 1.0f) {
        // Increase window size
        if (congestionState == CongestionState::SLOW_START) {
            targetWindow *= 2.0f;
        } else {
            targetWindow += 1.0f;
        }
    } else if (packetLoss > 5.0f) {
        // Decrease window size
        targetWindow *= 0.5f;
    }
    
    params.congestionWindow = clamp(
        targetWindow,
        MIN_CONGESTION_WINDOW,
        MAX_CONGESTION_WINDOW
    );
}

void AdaptiveNetworkControl::updateMetricsHistory(
    float rtt, float packetLoss, float bandwidth)
{
    metricsHistory[historyIndex] = {
        rtt,
        packetLoss,
        bandwidth,
        std::chrono::steady_clock::now()
    };
    
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

bool AdaptiveNetworkControl::detectNetworkDegradation() const {
    if (historyIndex < 2) return false;
    
    size_t prevIndex = (historyIndex + HISTORY_SIZE - 1) % HISTORY_SIZE;
    size_t prevPrevIndex = (historyIndex + HISTORY_SIZE - 2) % HISTORY_SIZE;
    
    const auto& current = metricsHistory[historyIndex];
    const auto& prev = metricsHistory[prevIndex];
    const auto& prevPrev = metricsHistory[prevPrevIndex];
    
    // Check for consistent degradation
    bool rttIncreasing = current.rtt > prev.rtt && prev.rtt > prevPrev.rtt;
    bool lossIncreasing = current.packetLoss > prev.packetLoss && 
                         prev.packetLoss > prevPrev.packetLoss;
    bool bandwidthDecreasing = current.bandwidth < prev.bandwidth && 
                              prev.bandwidth < prevPrev.bandwidth;
    
    return (rttIncreasing && current.rtt > 200.0f) ||
           (lossIncreasing && current.packetLoss > 5.0f) ||
           (bandwidthDecreasing && current.bandwidth < MIN_TRANSMISSION_RATE);
} 