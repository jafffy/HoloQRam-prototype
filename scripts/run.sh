#!/bin/bash

# Function to display usage
usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -s, --server       Run server"
    echo "  -c, --client       Run normal client"
    echo "  -o, --offscreen    Run offscreen client"
    echo "  -h, --help         Display this help message"
    exit 1
}

# Default values
RUN_SERVER=0
RUN_CLIENT=0
RUN_OFFSCREEN=0

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--server)
            RUN_SERVER=1
            shift
            ;;
        -c|--client)
            RUN_CLIENT=1
            shift
            ;;
        -o|--offscreen)
            RUN_OFFSCREEN=1
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

# If no options provided, show usage
if [[ $RUN_SERVER -eq 0 && $RUN_CLIENT -eq 0 && $RUN_OFFSCREEN -eq 0 ]]; then
    usage
fi

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi

# Build the project
cd build
cmake ..
make -j4

# Run components based on options
if [ $RUN_SERVER -eq 1 ]; then
    echo "Starting server..."
    ./server &
    SERVER_PID=$!
    sleep 2  # Wait for server to start
fi

if [ $RUN_CLIENT -eq 1 ]; then
    echo "Starting normal client..."
    ./client &
    CLIENT_PID=$!
fi

if [ $RUN_OFFSCREEN -eq 1 ]; then
    echo "Starting offscreen client..."
    ./offscreen_client &
    OFFSCREEN_PID=$!
fi

# Wait for Ctrl+C
trap 'kill $SERVER_PID $CLIENT_PID $OFFSCREEN_PID 2>/dev/null' SIGINT SIGTERM
wait
