#include "VolumetricClient.hpp"
#include <glm/gtc/matrix_transform.hpp>

Camera::Camera()
    : position(0.0f, 0.0f, 3.0f)
    , front(0.0f, 0.0f, -1.0f)
    , up(0.0f, 1.0f, 0.0f)
    , yaw(-90.0f)
    , pitch(0.0f)
{
}

void Camera::updateRotation(float newYaw, float newPitch) {
    yaw = newYaw;
    pitch = newPitch;

    // Constrain pitch
    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;

    // Calculate new front vector
    glm::vec3 newFront;
    newFront.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    newFront.y = sin(glm::radians(pitch));
    newFront.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(newFront);
}

glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(position, position + front, up);
} 