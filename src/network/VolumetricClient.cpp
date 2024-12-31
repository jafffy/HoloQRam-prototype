#include "network/VolumetricClient.hpp"
#include "graphics/RenderManager.hpp"
#include "core/Camera.hpp"
#include <iostream>

VolumetricClient::VolumetricClient()
    : BaseClient()
    , lastX(640.0f)
    , lastY(360.0f)
    , firstMouse(true)
{
    initializeGraphics();
}

VolumetricClient::~VolumetricClient() {
    cleanupGraphics();
}

void VolumetricClient::initializeGraphics() {
    // Initialize GLFW
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    window = glfwCreateWindow(1280, 720, "Volumetric Video Client", NULL, NULL);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(window);
    glfwSetWindowUserPointer(window, this);
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    try {
        // Initialize graphics components
        camera = std::make_unique<Camera>();
        renderManager = std::make_unique<RenderManager>(window, camera.get(), networkManager.get());
        renderManager->initialize();
    }
    catch (const std::exception& e) {
        // Clean up in case of initialization failure
        renderManager.reset();
        camera.reset();
        
        if (window) {
            glfwDestroyWindow(window);
        }
        glfwTerminate();
        
        throw;
    }
}

void VolumetricClient::cleanupGraphics() {
    renderManager.reset();
    camera.reset();

    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

void VolumetricClient::mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    VolumetricClient* client = static_cast<VolumetricClient*>(glfwGetWindowUserPointer(window));
    client->processMouse(xpos, ypos);
}

void VolumetricClient::processMouse(double xpos, double ypos) {
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
        return;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;

    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    float newYaw = camera->getYaw() + xoffset;
    float newPitch = camera->getPitch() + yoffset;

    camera->updateRotation(newYaw, newPitch);
}

void VolumetricClient::processInput() {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    float cameraSpeed = 0.05f;
    glm::vec3 position = camera->getPosition();
    glm::vec3 front = camera->getFront();
    glm::vec3 up = camera->getUp();

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        position += cameraSpeed * front;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        position -= cameraSpeed * front;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        position -= glm::normalize(glm::cross(front, up)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        position += glm::normalize(glm::cross(front, up)) * cameraSpeed;

    camera->updatePosition(position);
}

void VolumetricClient::run() {
    std::vector<float> currentVertices;

    while (!glfwWindowShouldClose(window) && !shouldStop) {
        processInput();

        // Get next frame if available
        if (getNextFrame(currentVertices)) {
            renderManager->render(currentVertices);
        } else {
            // Render previous frame if no new frame is available
            renderManager->render(currentVertices);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
} 