#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Shader sources
const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;
    out vec3 Color;
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
        Color = aColor;
        gl_PointSize = 5.0;
    }
)";

const char* fragmentShaderSource = R"(
    #version 330 core
    in vec3 Color;
    out vec4 FragColor;
    void main() {
        FragColor = vec4(Color, 1.0);
    }
)";

class VolumetricClient {
public:
    VolumetricClient() : window(nullptr), shaderProgram(0), 
                        cameraPos(0.0f, 0.0f, 3.0f),
                        cameraFront(0.0f, 0.0f, -1.0f),
                        cameraUp(0.0f, 1.0f, 0.0f),
                        lastX(400), lastY(300), yaw(-90.0f), pitch(0.0f),
                        firstMouse(true) {
        setupSocket();
        initializeGL();
        setupShaders();
        
        // Start receiving thread
        receiveThread = std::thread(&VolumetricClient::receiveData, this);
    }

    ~VolumetricClient() {
        if (receiveThread.joinable()) {
            receiveThread.join();
        }
        glfwTerminate();
        close(sockfd);
    }

    void run() {
        while (!glfwWindowShouldClose(window)) {
            render();
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

private:
    GLFWwindow* window;
    unsigned int shaderProgram;
    unsigned int VBO, VAO;
    int sockfd;
    struct sockaddr_in serverAddr;
    std::thread receiveThread;
    std::mutex cloudMutex;
    std::vector<float> vertices;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> decompressor;

    // Camera variables
    glm::vec3 cameraPos;
    glm::vec3 cameraFront;
    glm::vec3 cameraUp;
    float lastX, lastY;
    float yaw, pitch;
    bool firstMouse;

    static void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
        VolumetricClient* client = static_cast<VolumetricClient*>(glfwGetWindowUserPointer(window));
        client->processMouse(xpos, ypos);
    }

    void processMouse(double xpos, double ypos) {
        if (firstMouse) {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos;
        lastX = xpos;
        lastY = ypos;

        float sensitivity = 0.1f;
        xoffset *= sensitivity;
        yoffset *= sensitivity;

        yaw += xoffset;
        pitch += yoffset;

        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;

        glm::vec3 front;
        front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.y = sin(glm::radians(pitch));
        front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        cameraFront = glm::normalize(front);
    }

    void processInput() {
        float cameraSpeed = 0.05f;
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            cameraPos += cameraSpeed * cameraFront;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            cameraPos -= cameraSpeed * cameraFront;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    }

    void setupSocket() {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            std::cerr << "Error creating socket" << std::endl;
            exit(1);
        }

        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(8766);
        serverAddr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "Error binding socket" << std::endl;
            exit(1);
        }
    }

    void initializeGL() {
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            exit(1);
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        window = glfwCreateWindow(1280, 720, "Volumetric Video Client", NULL, NULL);
        if (!window) {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            exit(1);
        }

        glfwMakeContextCurrent(window);
        glfwSetWindowUserPointer(window, this);
        glfwSetCursorPosCallback(window, mouseCallback);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            std::cerr << "Failed to initialize GLAD" << std::endl;
            exit(1);
        }

        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_DEPTH_TEST);
    }

    void setupShaders() {
        // Vertex shader
        unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        glCompileShader(vertexShader);

        // Fragment shader
        unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        glCompileShader(fragmentShader);

        // Shader program
        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        // Create buffers
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
    }

    void receiveData() {
        char buffer[65507];
        std::vector<char> receivedData;
        uint32_t expectedSize = 0;
        bool waitingForSize = true;
        
        while (true) {
            ssize_t bytesReceived = recv(sockfd, buffer, sizeof(buffer), 0);
            if (bytesReceived > 0) {
                if (waitingForSize) {
                    // First packet contains the size
                    if (bytesReceived >= sizeof(uint32_t)) {
                        expectedSize = *reinterpret_cast<uint32_t*>(buffer);
                        receivedData.reserve(expectedSize);
                        waitingForSize = false;
                    }
                } else {
                    // Append received data
                    receivedData.insert(receivedData.end(), buffer, buffer + bytesReceived);
                    
                    // Check if we have received all data
                    if (receivedData.size() >= expectedSize) {
                        try {
                            // Create stringstream from received data
                            std::stringstream compressedData;
                            compressedData.write(receivedData.data(), expectedSize);
                            
                            // Try to decompress and update vertices
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                            decompressor.decodePointCloud(compressedData, cloud);
                            
                            std::lock_guard<std::mutex> lock(cloudMutex);
                            vertices.clear();
                            for (const auto& point : cloud->points) {
                                // Position
                                vertices.push_back(point.x);
                                vertices.push_back(point.y);
                                vertices.push_back(point.z);
                                // Color
                                vertices.push_back(point.r / 255.0f);
                                vertices.push_back(point.g / 255.0f);
                                vertices.push_back(point.b / 255.0f);
                            }
                        } catch (const std::exception& e) {
                            std::cerr << "Error processing point cloud: " << e.what() << std::endl;
                        }
                        
                        // Reset for next frame
                        receivedData.clear();
                        waitingForSize = true;
                    }
                }
            }
        }
    }

    void render() {
        processInput();
        
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);

        // Set up transformation matrices
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1280.0f / 720.0f, 0.1f, 100.0f);

        // Set uniforms
        unsigned int modelLoc = glGetUniformLocation(shaderProgram, "model");
        unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");
        unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");

        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

        // Update point cloud data
        std::lock_guard<std::mutex> lock(cloudMutex);
        if (!vertices.empty()) {
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
    }
};

int main() {
    VolumetricClient client;
    client.run();
    return 0;
} 