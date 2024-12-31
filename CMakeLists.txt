cmake_minimum_required(VERSION 3.10)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(holoQRam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PCL 1.8 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(Freetype REQUIRED)

# Add glad library
add_library(glad
    external/glad/src/glad.c
)
target_include_directories(glad PUBLIC external/glad/include)

# Common source files for compression schemes
set(COMPRESSION_SOURCES
    src/network/CompressionScheme.cpp
    src/network/OctreeCompression.cpp
    src/network/RLECompression.cpp
)

# Client source files
set(CLIENT_SOURCES
    src/main.cpp
    src/network/VolumetricClient.cpp
    src/network/BaseClient.cpp
    src/core/Camera.cpp
    src/network/NetworkManager.cpp
    src/network/DecompressionManager.cpp
    src/graphics/RenderManager.cpp
    src/graphics/TextRenderer.cpp
    ${COMPRESSION_SOURCES}
)

# Off-screen client source files
set(OFFSCREEN_CLIENT_SOURCES
    src/offscreen_main.cpp
    src/network/OffscreenClient.cpp
    src/network/BaseClient.cpp
    src/network/NetworkManager.cpp
    src/network/DecompressionManager.cpp
    ${COMPRESSION_SOURCES}
)

# Server source files
set(SERVER_SOURCES
    src/server/src/main.cpp
    src/server/src/VolumetricServer.cpp
    ${COMPRESSION_SOURCES}
)

# Common include directories
set(COMMON_INCLUDE_DIRS
    ${PCL_INCLUDE_DIRS}
    include
)

# Client executable
add_executable(client ${CLIENT_SOURCES})
target_include_directories(client PRIVATE
    ${COMMON_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${FREETYPE_INCLUDE_DIRS}
    include/core
    include/graphics
    include/network
)
target_link_libraries(client PRIVATE
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    glfw
    glad
    pthread
)

# Off-screen client executable
add_executable(offscreen_client ${OFFSCREEN_CLIENT_SOURCES})
target_include_directories(offscreen_client PRIVATE
    ${COMMON_INCLUDE_DIRS}
    include/network
)
target_link_libraries(offscreen_client PRIVATE
    ${PCL_LIBRARIES}
    glfw
    pthread
)

# Server executable
add_executable(server ${SERVER_SOURCES})
target_include_directories(server PRIVATE
    ${COMMON_INCLUDE_DIRS}
    include/server
    include/network
)
target_link_libraries(server PRIVATE
    ${PCL_LIBRARIES}
    pthread
) 