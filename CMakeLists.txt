cmake_minimum_required(VERSION 3.10)
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

# Client source files
set(CLIENT_SOURCES
    src/main.cpp
    src/network/VolumetricClient.cpp
    src/core/Camera.cpp
    src/network/NetworkManager.cpp
    src/network/DecompressionManager.cpp
    src/graphics/RenderManager.cpp
    src/graphics/TextRenderer.cpp
)

# Server source files
set(SERVER_SOURCES
    src/server/src/main.cpp
    src/server/src/VolumetricServer.cpp
)

# Client executable
add_executable(client ${CLIENT_SOURCES})
target_include_directories(client PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${FREETYPE_INCLUDE_DIRS}
    include
    include/core
    include/graphics
    include/network
)
target_link_libraries(client PRIVATE
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    glfw
    glad
    ${FREETYPE_LIBRARIES}
)

# Server executable
add_executable(server ${SERVER_SOURCES})
target_include_directories(server PRIVATE
    ${PCL_INCLUDE_DIRS}
    include
    include/server
)
target_link_libraries(server PRIVATE
    ${PCL_LIBRARIES}
) 