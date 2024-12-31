# HoloQRam - Volumetric Video Streaming

A minimal C++ implementation of volumetric video streaming using Point Cloud Library (PCL) and OpenGL.

## Features
- Real-time point cloud streaming using UDP
- OpenGL-based 3D visualization
- Point cloud compression using PCL's octree compression
- Multi-threaded client for smooth rendering

## Prerequisites

### Ubuntu/Debian
```bash
# Install required packages
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    libpcl-dev \
    libglfw3-dev \
    libopencv-dev
```

## Building

```bash
# Create build directory
mkdir build
cd build

# Configure and build
cmake ..
make -j4
```

## Running

1. Start the server:
```bash
./server
```

2. In a new terminal, start the client:
```bash
./client
```

## Project Structure

```
holoQRam/
├── include/                    # Header files
│   ├── core/                  # Core functionality headers
│   ├── graphics/              # Rendering related headers
│   └── network/               # Network related headers
├── src/                       # Source files
│   ├── core/                  # Core functionality (Camera, etc.)
│   ├── graphics/              # Rendering related code
│   ├── network/               # Network and streaming code
│   └── main.cpp              # Application entry point
├── external/                  # Third-party dependencies
├── scripts/                   # Build and utility scripts
├── tests/                     # Unit tests
├── examples/                  # Example programs
├── docs/                      # Documentation
└── CMakeLists.txt            # Build configuration
```

### Component Overview
- **Core**: Basic functionality like camera controls and utilities
- **Graphics**: OpenGL rendering, shaders, and text rendering
- **Network**: Point cloud streaming, compression, and network management

## Current Limitations
- Basic point cloud visualization
- Random point cloud generation (no real data capture yet)
- Local network streaming only
- Fixed point size rendering

## Next Steps
- Add real point cloud capture using depth cameras
- Implement more efficient compression
- Add camera controls
- Support for mesh streaming
- Network optimization for remote streaming 