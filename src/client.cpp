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
#include <iomanip>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <sstream>
#include <map>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <deque>
#include <numeric>

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

// Add text rendering shaders
const char* textVertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec4 vertex; // <vec2 pos, vec2 tex>
    out vec2 TexCoords;
    
    uniform mat4 projection;
    
    void main() {
        gl_Position = projection * vec4(vertex.xy, 0.0, 1.0);
        TexCoords = vertex.zw;
    }
)";

const char* textFragmentShaderSource = R"(
    #version 330 core
    in vec2 TexCoords;
    out vec4 color;
    
    uniform sampler2D text;
    uniform vec3 textColor;
    
    void main() {
        vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);
        color = vec4(textColor, 1.0) * sampled;
    }
)";

// Text rendering character structure
struct Character {
    unsigned int TextureID;  // ID handle of the glyph texture
    glm::ivec2   Size;      // Size of glyph
    glm::ivec2   Bearing;   // Offset from baseline to left/top of glyph
    unsigned int Advance;    // Offset to advance to next glyph
};

class VolumetricClient {
public:
    VolumetricClient() : window(nullptr), shaderProgram(0), 
                        cameraPos(0.0f, 0.0f, 3.0f),
                        cameraFront(0.0f, 0.0f, -1.0f),
                        cameraUp(0.0f, 1.0f, 0.0f),
                        lastX(400), lastY(300), yaw(-90.0f), pitch(0.0f),
                        firstMouse(true),
                        totalBytesReceived(0),
                        lastBandwidthCheck(std::chrono::steady_clock::now()),
                        currentBandwidth(0.0),
                        currentRTT(0.0),
                        lastPacketSentTime(std::chrono::steady_clock::now()) {
        setupSocket();
        initializeGL();
        setupShaders();
        setupTextRendering();
        
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

    // Bandwidth measurement variables
    std::atomic<uint64_t> totalBytesReceived;
    std::chrono::steady_clock::time_point lastBandwidthCheck;
    std::atomic<double> currentBandwidth;  // in MB/s
    std::mutex bandwidthMutex;

    // RTT measurement variables
    std::chrono::steady_clock::time_point lastPacketSentTime;
    std::atomic<double> currentRTT;
    std::mutex rttMutex;
    std::deque<double> rttHistory;
    static const size_t RTT_HISTORY_SIZE = 10;  // Keep last 10 measurements for averaging

    // Text rendering members
    unsigned int textShaderProgram;
    unsigned int textVAO, textVBO;
    std::map<char, Character> characters;
    
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

    void setupTextRendering() {
        // Initialize FreeType
        FT_Library ft;
        if (FT_Init_FreeType(&ft)) {
            std::cerr << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
            return;
        }

        // Load font
        FT_Face face;
        if (FT_New_Face(ft, "/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 0, &face)) {
            std::cerr << "ERROR::FREETYPE: Failed to load font" << std::endl;
            return;
        }

        // Set font size (0 width means dynamically calculate width based on height)
        FT_Set_Pixel_Sizes(face, 0, 48); // Increased font size to 48 pixels

        // Enable blending for text
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // Disable byte-alignment restriction

        // Load first 128 characters of ASCII set
        for (unsigned char c = 0; c < 128; c++) {
            if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
                std::cerr << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
                continue;
            }

            // Generate texture
            unsigned int texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(
                GL_TEXTURE_2D,
                0,
                GL_RED,
                face->glyph->bitmap.width,
                face->glyph->bitmap.rows,
                0,
                GL_RED,
                GL_UNSIGNED_BYTE,
                face->glyph->bitmap.buffer
            );

            // Set texture options
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            // Store character for later use
            Character character = {
                texture,
                glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
                glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
                static_cast<unsigned int>(face->glyph->advance.x)
            };
            characters.insert(std::pair<char, Character>(c, character));
        }

        // Cleanup FreeType resources
        FT_Done_Face(face);
        FT_Done_FreeType(ft);

        // Configure VAO/VBO for texture quads
        glGenVertexArrays(1, &textVAO);
        glGenBuffers(1, &textVBO);
        glBindVertexArray(textVAO);
        glBindBuffer(GL_ARRAY_BUFFER, textVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        // Create and compile the text shader program
        unsigned int textVertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(textVertexShader, 1, &textVertexShaderSource, NULL);
        glCompileShader(textVertexShader);

        unsigned int textFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(textFragmentShader, 1, &textFragmentShaderSource, NULL);
        glCompileShader(textFragmentShader);

        textShaderProgram = glCreateProgram();
        glAttachShader(textShaderProgram, textVertexShader);
        glAttachShader(textShaderProgram, textFragmentShader);
        glLinkProgram(textShaderProgram);

        glDeleteShader(textVertexShader);
        glDeleteShader(textFragmentShader);
    }

    void renderText(const std::string& text, float x, float y, float scale, glm::vec3 color) {
        glUseProgram(textShaderProgram);
        glm::mat4 projection = glm::ortho(0.0f, static_cast<float>(1280), 0.0f, static_cast<float>(720));
        glUniformMatrix4fv(glGetUniformLocation(textShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniform3f(glGetUniformLocation(textShaderProgram, "textColor"), color.x, color.y, color.z);
        
        glActiveTexture(GL_TEXTURE0);
        glBindVertexArray(textVAO);

        float originalX = x;
        for (char c : text) {
            Character ch = characters[c];

            float xpos = x + ch.Bearing.x * scale;
            float ypos = y - (ch.Size.y - ch.Bearing.y) * scale;

            float w = ch.Size.x * scale;
            float h = ch.Size.y * scale;

            float vertices[6][4] = {
                { xpos,     ypos + h,   0.0f, 0.0f },            
                { xpos,     ypos,       0.0f, 1.0f },
                { xpos + w, ypos,       1.0f, 1.0f },

                { xpos,     ypos + h,   0.0f, 0.0f },
                { xpos + w, ypos,       1.0f, 1.0f },
                { xpos + w, ypos + h,   1.0f, 0.0f }           
            };

            glBindTexture(GL_TEXTURE_2D, ch.TextureID);
            glBindBuffer(GL_ARRAY_BUFFER, textVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glDrawArrays(GL_TRIANGLES, 0, 6);

            x += (ch.Advance >> 6) * scale;
        }
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void updateBandwidth(size_t newBytes) {
        totalBytesReceived += newBytes;
        
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastBandwidthCheck).count();
        
        // Update bandwidth every second
        if (duration >= 1000) {
            std::lock_guard<std::mutex> lock(bandwidthMutex);
            double seconds = duration / 1000.0;
            currentBandwidth = (totalBytesReceived / (1024.0 * 1024.0)) / seconds; // Convert to MB/s
            
            // Print bandwidth
            std::cout << "\rBandwidth: " << std::fixed << std::setprecision(2) 
                      << currentBandwidth << " MB/s" << std::flush;
            
            // Reset counters
            totalBytesReceived = 0;
            lastBandwidthCheck = now;
        }
    }

    void updateRTT() {
        auto now = std::chrono::steady_clock::now();
        double rtt = std::chrono::duration_cast<std::chrono::microseconds>(now - lastPacketSentTime).count() / 1000.0; // Convert to milliseconds

        std::lock_guard<std::mutex> lock(rttMutex);
        rttHistory.push_back(rtt);
        if (rttHistory.size() > RTT_HISTORY_SIZE) {
            rttHistory.pop_front();
        }
        
        // Calculate average RTT
        currentRTT = std::accumulate(rttHistory.begin(), rttHistory.end(), 0.0) / rttHistory.size();
    }

    void receiveData() {
        char buffer[65507];
        std::vector<char> receivedData;
        uint32_t expectedSize = 0;
        bool waitingForSize = true;
        
        while (true) {
            ssize_t bytesReceived = recv(sockfd, buffer, sizeof(buffer), 0);
            if (bytesReceived > 0) {
                // Update bandwidth and RTT measurements
                updateBandwidth(bytesReceived);
                updateRTT();

                if (waitingForSize) {
                    // First packet contains the size
                    if (bytesReceived >= sizeof(uint32_t)) {
                        expectedSize = *reinterpret_cast<uint32_t*>(buffer);
                        receivedData.reserve(expectedSize);
                        waitingForSize = false;
                        
                        // Update last packet sent time for next RTT measurement
                        lastPacketSentTime = std::chrono::steady_clock::now();
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

        // Render network statistics
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) 
           << "Bandwidth: " << currentBandwidth << " MB/s\n"
           << "RTT: " << currentRTT << " ms";
        
        // Render each line of text
        std::string line;
        float y_position = 650.0f;
        while (std::getline(ss, line)) {
            renderText(line, 25.0f, y_position, 0.75f, glm::vec3(1.0f, 1.0f, 0.0f));
            y_position -= 50.0f;  // Move down for next line
        }
    }
};

int main() {
    VolumetricClient client;
    client.run();
    return 0;
} 