#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "network/BaseClient.hpp"

// Forward declarations
class RenderManager;
class Camera;

class VolumetricClient : public BaseClient {
public:
    VolumetricClient();
    ~VolumetricClient() override;
    
    void run() override;

private:
    GLFWwindow* window;
    std::unique_ptr<Camera> camera;
    std::unique_ptr<RenderManager> renderManager;
    
    // Mouse handling
    float lastX;
    float lastY;
    bool firstMouse;
    
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    void processMouse(double xpos, double ypos);
    void processInput();
    void initializeGraphics();
    void cleanupGraphics();
}; 