#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <ft2build.h>
#include FT_FREETYPE_H

#include <string>
#include <map>
#include <deque>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include <chrono>

#include "VolumetricClient.hpp"  // For Camera class definition

struct Character {
    unsigned int TextureID;
    glm::ivec2 Size;
    glm::ivec2 Bearing;
    unsigned int Advance;
};

class TextRenderer {
public:
    TextRenderer();
    ~TextRenderer();

    void initialize();
    void renderText(const std::string& text, float x, float y, float scale, glm::vec3 color);

private:
    unsigned int textShaderProgram;
    unsigned int textVAO, textVBO;
    std::map<char, Character> characters;

    void setupShader();
    void loadFont();
};

class RenderManager {
public:
    RenderManager(GLFWwindow* window, Camera* camera, NetworkManager* networkManager);
    ~RenderManager();

    void initialize();
    void render(const std::vector<float>& vertices);
    void updateFPS();
    double getCurrentFPS() const { return currentFPS; }

private:
    void setupShaders();
    void setupBuffers();
    void renderStats();

    GLFWwindow* window;
    Camera* camera;
    NetworkManager* networkManager;  // Non-owning pointer
    std::unique_ptr<TextRenderer> textRenderer;

    // OpenGL resources
    unsigned int shaderProgram;
    unsigned int VAO, VBO;

    // FPS tracking
    std::chrono::steady_clock::time_point lastFrameTime;
    std::atomic<double> currentFPS;
    std::deque<double> fpsHistory;
    std::mutex fpsMutex;
    static constexpr size_t FPS_HISTORY_SIZE = 10;
}; 