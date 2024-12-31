#pragma once

// Standard library includes
#include <cstdint>
#include <chrono>
#include <vector>

namespace hologram {

// Network constants
constexpr uint16_t DEFAULT_PORT = 8765;
constexpr size_t MAX_PACKET_SIZE = 65507;  // Max UDP packet size
constexpr size_t MAX_PAYLOAD_SIZE = 65000;  // Leave room for headers
constexpr size_t CHUNK_HEADER_SIZE = 24;    // Increased for sequence numbers and timestamps

// Timing constants
constexpr uint32_t ACK_TIMEOUT_MS = 50;     // Time to wait for ACK before retransmission
constexpr uint32_t MAX_RETRIES = 3;         // Maximum retransmission attempts
constexpr uint32_t CLEANUP_INTERVAL_MS = 1000; // Interval for cleaning up old packets

// Packet types
enum class PacketType : uint8_t {
    POINT_CLOUD_DATA = 0,
    VIEWPORT_UPDATE = 1,
    ACK = 2,
    NACK = 3,
    RETRANSMIT_REQUEST = 4
};

// Viewport information
#pragma pack(push, 1)
struct ViewportInfo {
    float position[3];   // x, y, z
    float rotation[3];   // pitch, yaw, roll in radians
    float fov;          // field of view in radians
    float aspectRatio;  // width / height
    float nearPlane;    // near clipping plane
    float farPlane;     // far clipping plane
};

// Packet headers
struct PacketHeader {
    PacketType type;
    uint32_t sequenceNumber;
    uint32_t timestamp;      // milliseconds since epoch
    uint32_t totalChunks;    // For multi-chunk messages
    uint32_t chunkIndex;     // Current chunk index
    uint32_t payloadSize;    // Size of the actual data
};

struct AckPacket {
    PacketType type;
    uint32_t sequenceNumber;
    uint32_t timestamp;
};

struct RetransmitRequest {
    PacketType type;
    uint32_t sequenceNumber;
    uint32_t chunkIndex;
};
#pragma pack(pop)

// Packet tracking
struct PacketInfo {
    std::chrono::steady_clock::time_point sendTime;
    uint32_t retryCount;
    bool acknowledged;
    std::vector<char> data;  // Original packet data for retransmission
};

// Sequence number utilities
inline uint32_t getNextSequenceNumber(uint32_t current) {
    return (current + 1) % UINT32_MAX;
}

inline bool isSequenceNewer(uint32_t seq1, uint32_t seq2) {
    // Handle wraparound
    const uint32_t HALF_MAX = UINT32_MAX / 2;
    return (seq1 > seq2 && seq1 - seq2 <= HALF_MAX) ||
           (seq1 < seq2 && seq2 - seq1 > HALF_MAX);
}

} // namespace hologram 