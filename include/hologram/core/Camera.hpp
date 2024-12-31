#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace hologram {

class Camera {
public:
    Camera();

    void updateRotation(float newYaw, float newPitch);
    void updatePosition(const glm::vec3& newPosition) { position = newPosition; }
    
    glm::mat4 getViewMatrix() const;
    
    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }
    const glm::vec3& getPosition() const { return position; }
    const glm::vec3& getFront() const { return front; }
    const glm::vec3& getUp() const { return up; }

private:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    float yaw;
    float pitch;
};

} // namespace hologram 