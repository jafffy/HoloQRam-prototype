#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <atomic>

// Forward declarations
class NetworkManager;
class RenderManager;
class DecompressionManager;
class TextRenderer;
class Camera;

class VolumetricClient {
public:
    VolumetricClient();
    ~VolumetricClient();
    
    void run();

private:
    void processInput();
    void processMouse(double xpos, double ypos);
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);

    // Core components
    std::unique_ptr<NetworkManager> networkManager;
    std::unique_ptr<RenderManager> renderManager;
    std::unique_ptr<DecompressionManager> decompressionManager;
    std::unique_ptr<Camera> camera;
    
    // Window management
    GLFWwindow* window;
    std::atomic<bool> shouldStop;

    // Mouse state
    float lastX, lastY;
    bool firstMouse;
}; 