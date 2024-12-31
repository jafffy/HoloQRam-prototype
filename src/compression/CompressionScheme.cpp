#include "hologram/compression/CompressionScheme.hpp"
#include "hologram/compression/OctreeCompression.hpp"
#include "hologram/compression/RLECompression.hpp"
#include <stdexcept>

std::unique_ptr<CompressionScheme> CompressionScheme::create(const std::string& scheme) {
    if (scheme == "octree") {
        return std::make_unique<OctreeCompression>();
    } else if (scheme == "rle") {
        return std::make_unique<RLECompression>();
    } else {
        throw std::runtime_error("Unknown compression scheme: " + scheme);
    }
} 