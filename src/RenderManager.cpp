#include "RenderManager.hpp"
#include "Shaders.hpp"
#include <iostream>
#include <numeric>
#include <sstream>
#include <iomanip>

RenderManager::RenderManager(GLFWwindow* window, Camera* camera, NetworkManager* networkManager)
    : window(window)
    , camera(camera)
    , networkManager(networkManager)
    , shaderProgram(0)
    , VAO(0)
    , VBO(0)
    , currentFPS(0.0)
    , lastFrameTime(std::chrono::steady_clock::now())
{
    textRenderer = std::make_unique<TextRenderer>();
}

RenderManager::~RenderManager() {
    if (VAO) glDeleteVertexArrays(1, &VAO);
    if (VBO) glDeleteBuffers(1, &VBO);
    if (shaderProgram) glDeleteProgram(shaderProgram);
}

void RenderManager::initialize() {
    setupShaders();
    setupBuffers();
    textRenderer->initialize();

    // Enable necessary OpenGL features
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void RenderManager::setupShaders() {
    // Vertex shader
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &Shaders::VERTEX_SHADER_SOURCE, NULL);
    glCompileShader(vertexShader);

    // Check vertex shader compilation
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        throw std::runtime_error("Vertex shader compilation failed: " + std::string(infoLog));
    }

    // Fragment shader
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &Shaders::FRAGMENT_SHADER_SOURCE, NULL);
    glCompileShader(fragmentShader);

    // Check fragment shader compilation
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        throw std::runtime_error("Fragment shader compilation failed: " + std::string(infoLog));
    }

    // Link shaders
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // Check linking
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        throw std::runtime_error("Shader program linking failed: " + std::string(infoLog));
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void RenderManager::setupBuffers() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
}

void RenderManager::updateFPS() {
    auto now = std::chrono::steady_clock::now();
    double frameTime = std::chrono::duration_cast<std::chrono::microseconds>(
        now - lastFrameTime).count() / 1000000.0; // Convert to seconds
    lastFrameTime = now;

    std::lock_guard<std::mutex> lock(fpsMutex);
    fpsHistory.push_back(1.0 / frameTime);
    if (fpsHistory.size() > FPS_HISTORY_SIZE) {
        fpsHistory.pop_front();
    }
    currentFPS = std::accumulate(fpsHistory.begin(), fpsHistory.end(), 0.0) / fpsHistory.size();
}

void RenderManager::render(const std::vector<float>& vertices) {
    updateFPS();

    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!vertices.empty()) {
        glUseProgram(shaderProgram);

        // Set up transformation matrices
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = camera->getViewMatrix();
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1280.0f / 720.0f, 0.1f, 100.0f);

        // Set uniforms
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

        // Update and bind vertex data
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // Color attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        // Draw points
        glDrawArrays(GL_POINTS, 0, vertices.size() / 6);
    }

    renderStats();
}

void RenderManager::renderStats() {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) 
       << "FPS: " << currentFPS << "\n"
       << "Bandwidth: " << networkManager->getCurrentBandwidth() << " MB/s\n"
       << "RTT: " << networkManager->getCurrentRTT() << " ms";
    
    // Render each line of text
    std::string line;
    float y_position = 650.0f;
    while (std::getline(ss, line)) {
        textRenderer->renderText(line, 25.0f, y_position, 0.75f, glm::vec3(1.0f, 1.0f, 0.0f));
        y_position -= 30.0f;  // Adjust spacing between lines
    }
} 