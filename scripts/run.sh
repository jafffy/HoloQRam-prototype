#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$SCRIPT_DIR/.."

# Check if build directory exists
if [ ! -d "$PROJECT_ROOT/build" ]; then
    echo "Build directory not found. Please build the project first."
    exit 1
fi

# Function to cleanup background processes on script exit
cleanup() {
    echo "Cleaning up processes..."
    kill $SERVER_PID 2>/dev/null
    kill $CLIENT_PID 2>/dev/null
    exit 0
}

# Set up cleanup trap
trap cleanup EXIT INT TERM

# Launch server in background
echo "Starting server..."
"$PROJECT_ROOT/build/server" &
SERVER_PID=$!

# Wait a moment for server to initialize
sleep 2

# Launch client
echo "Starting client..."
"$PROJECT_ROOT/build/client" &
CLIENT_PID=$!

# Wait for either process to exit
wait $SERVER_PID $CLIENT_PID 
