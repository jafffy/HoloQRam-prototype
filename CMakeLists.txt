cmake_minimum_required(VERSION 3.10)
project(holoQRam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Freetype REQUIRED)

# Add external dependencies
include(FetchContent)
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG master
)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG master
)

FetchContent_MakeAvailable(glad glm)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${glad_SOURCE_DIR}/include)
include_directories(${glm_SOURCE_DIR})
include_directories(${FREETYPE_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Server executable
add_executable(server src/server.cpp)
target_link_libraries(server
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

# Client executable
add_executable(client src/client.cpp)
target_link_libraries(client
    glad
    ${OPENGL_LIBRARIES}
    glfw
    ${PCL_LIBRARIES}
    ${FREETYPE_LIBRARIES}
) 