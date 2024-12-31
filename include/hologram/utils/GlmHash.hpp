#pragma once

#include <glm/glm.hpp>
#include <functional>

namespace std {

template<>
struct hash<glm::vec3> {
    size_t operator()(const glm::vec3& v) const noexcept {
        // Simple hash function that combines the components
        return static_cast<size_t>(
            v.x * 73856093 ^
            v.y * 19349663 ^
            v.z * 83492791
        );
    }
};

} // namespace std 