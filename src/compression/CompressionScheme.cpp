#include "hologram/compression/CompressionScheme.hpp"
#include "hologram/compression/OctreeCompression.hpp"
#include "hologram/compression/RLECompression.hpp"
#include "hologram/compression/ViVoCompression.hpp"
#include <stdexcept>

namespace hologram {

std::unique_ptr<CompressionScheme> CompressionScheme::create(const std::string& scheme) {
    if (scheme == "octree") {
        return std::make_unique<hologram::OctreeCompression>();
    } else if (scheme == "rle") {
        return std::make_unique<hologram::RLECompression>();
    } else if (scheme == "vivo") {
        return std::make_unique<hologram::ViVoCompression>();
    } else {
        throw std::runtime_error("Unknown compression scheme: " + scheme);
    }
}

} // namespace hologram 