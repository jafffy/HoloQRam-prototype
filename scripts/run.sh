#!/bin/bash

# Default values
COMPRESSION="octree"
CLIENT_TYPE="normal"  # can be "normal" or "offscreen"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --compression)
            COMPRESSION="$2"
            shift 2
            ;;
        --client)
            CLIENT_TYPE="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --compression <scheme>  Compression scheme to use (octree or rle, default: octree)"
            echo "  --client <type>        Client type to run (normal or offscreen, default: normal)"
            echo "  --help                 Show this help message"
            exit 0
            ;;
        *)
            echo "Error: Unknown argument '$1'"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Validate compression scheme
if [[ "$COMPRESSION" != "octree" && "$COMPRESSION" != "rle" ]]; then
    echo "Error: Invalid compression scheme. Use 'octree' or 'rle'."
    exit 1
fi

# Validate client type
if [[ "$CLIENT_TYPE" != "normal" && "$CLIENT_TYPE" != "offscreen" ]]; then
    echo "Error: Invalid client type. Use 'normal' or 'offscreen'."
    exit 1
fi

# Build the project
mkdir -p build
cd build
cmake ..
make -j4

# Function to cleanup processes
cleanup() {
    echo "Shutting down gracefully..."
    kill $SERVER_PID $CLIENT_PID 2>/dev/null
    wait $SERVER_PID $CLIENT_PID 2>/dev/null
    exit 0
}

# Set up trap for Ctrl+C and termination signals
trap cleanup SIGINT SIGTERM

# Start server
echo "Starting server with compression scheme: $COMPRESSION"
./server --compression "$COMPRESSION" &
SERVER_PID=$!

sleep 1  # Give server time to start

# Start appropriate client
if [[ "$CLIENT_TYPE" == "normal" ]]; then
    echo "Starting normal client with compression scheme: $COMPRESSION"
    ./client --compression "$COMPRESSION" &
else
    echo "Starting offscreen client with compression scheme: $COMPRESSION"
    ./offscreen_client --compression "$COMPRESSION" &
fi
CLIENT_PID=$!

# Wait for any signal
while true; do
    sleep 1
done
