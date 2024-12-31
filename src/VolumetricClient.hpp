#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <vector>
#include <condition_variable>
#include <netinet/in.h>

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

class Camera {
public:
    Camera();
    
    void updatePosition(const glm::vec3& newPos) { position = newPos; }
    void updateFront(const glm::vec3& newFront) { front = newFront; }
    void updateRotation(float newYaw, float newPitch);
    
    glm::mat4 getViewMatrix() const;
    glm::vec3 getPosition() const { return position; }
    glm::vec3 getFront() const { return front; }
    glm::vec3 getUp() const { return up; }
    
    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }

private:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    float yaw;
    float pitch;
};

class NetworkManager {
public:
    NetworkManager(DecompressionManager* decompManager);
    ~NetworkManager();
    
    void start();
    void stop();
    
    double getCurrentBandwidth() const { return currentBandwidth; }
    double getCurrentRTT() const { return currentRTT; }

private:
    void setupSocket();
    void receiveData();
    void updateBandwidth(size_t newBytes);
    void updateRTT();
    
    DecompressionManager* decompManager;  // Non-owning pointer
    int sockfd;
    struct sockaddr_in serverAddr;
    std::thread receiveThread;
    
    std::atomic<uint64_t> totalBytesReceived;
    std::chrono::steady_clock::time_point lastBandwidthCheck;
    std::chrono::steady_clock::time_point lastPacketTime;
    std::atomic<double> currentBandwidth;
    std::atomic<double> currentRTT;
    
    std::mutex rttMutex;
    std::deque<double> rttHistory;
    static constexpr size_t RTT_HISTORY_SIZE = 10;
    
    std::atomic<bool> shouldStop;
};

class DecompressionManager {
public:
    static constexpr size_t MAX_COMPRESSED_FRAMES = 30;
    static constexpr size_t MAX_DECOMPRESSED_FRAMES = 10;

    DecompressionManager();
    ~DecompressionManager();
    
    void start();
    void stop();
    
    void addCompressedFrame(std::vector<char>&& compressedData);
    bool getNextDecompressedFrame(std::vector<float>& vertices);

private:
    void decompressFrames();
    
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> decompressor;
    std::thread decompressionThread;
    
    std::mutex compressedFramesMutex;
    std::mutex decompressedFramesMutex;
    std::condition_variable compressedFramesCV;
    std::condition_variable decompressedFramesCV;
    
    std::deque<std::vector<char>> compressedFrames;
    std::deque<std::vector<float>> decompressedFrames;
    
    std::atomic<bool> shouldStop;
}; 