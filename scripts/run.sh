#!/bin/bash

# Default values
COMPRESSION="vivo"
CELL_SIZE="1.0"
VERBOSE=true
MODE="both"  # Can be "server", "client", or "both"

# Function to print usage
print_usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -m, --mode MODE         Set mode (server/client/both) (default: both)"
    echo "  -c, --compression SCHEME Set compression scheme (default: vivo)"
    echo "  -s, --cell-size SIZE    Set cell size (default: 1.0)"
    echo "  -q, --quiet             Disable verbose logging"
    echo "  -h, --help              Show this help message"
}

# Function to kill existing processes
kill_existing_processes() {
    echo "Checking for existing processes..."
    
    # Kill existing server if running
    if [ -f "/tmp/hologram_server.pid" ]; then
        OLD_PID=$(cat /tmp/hologram_server.pid)
        if ps -p $OLD_PID > /dev/null; then
            echo "Killing existing server process (PID: $OLD_PID)"
            kill $OLD_PID
            sleep 1
        fi
        rm -f /tmp/hologram_server.pid
    fi
    
    # Kill existing client if running
    if [ -f "/tmp/hologram_client.pid" ]; then
        OLD_PID=$(cat /tmp/hologram_client.pid)
        if ps -p $OLD_PID > /dev/null; then
            echo "Killing existing client process (PID: $OLD_PID)"
            kill $OLD_PID
            sleep 1
        fi
        rm -f /tmp/hologram_client.pid
    fi
}

# Parse command line arguments
while [ $# -gt 0 ]; do
    key="$1"
    case $key in
        -h|--help)
            print_usage
            exit 0
            ;;
        -q|--quiet)
            VERBOSE=false
            shift
            ;;
        -m|--mode)
            if [ $# -gt 1 ]; then
                MODE="$2"
                if [[ "$MODE" != "server" && "$MODE" != "client" && "$MODE" != "both" ]]; then
                    echo "Error: mode must be 'server', 'client', or 'both'" >&2
                    exit 1
                fi
                shift 2
            else
                echo "Error: --mode requires a value" >&2
                exit 1
            fi
            ;;
        -c|--compression)
            if [ $# -gt 1 ]; then
                COMPRESSION="$2"
                shift 2
            else
                echo "Error: --compression requires a scheme" >&2
                exit 1
            fi
            ;;
        -s|--cell-size)
            if [ $# -gt 1 ]; then
                CELL_SIZE="$2"
                shift 2
            else
                echo "Error: --cell-size requires a value" >&2
                exit 1
            fi
            ;;
        *)
            echo "Error: Unknown option: $1" >&2
            print_usage
            exit 1
            ;;
    esac
done

# Build command arguments
CMD_ARGS=""
if [ "$VERBOSE" = "false" ]; then
    CMD_ARGS="$CMD_ARGS -q"
fi

if [ -n "$COMPRESSION" ]; then
    CMD_ARGS="$CMD_ARGS -c $COMPRESSION"
fi

if [ -n "$CELL_SIZE" ]; then
    CMD_ARGS="$CMD_ARGS -s $CELL_SIZE"
fi

# Get the directory of the script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Kill any existing processes
kill_existing_processes

# Function to run server
run_server() {
    echo "Starting server with options: $CMD_ARGS"
    "$PROJECT_ROOT/build/server" $CMD_ARGS &
    SERVER_PID=$!
    echo $SERVER_PID > /tmp/hologram_server.pid
    echo "Server started with PID: $SERVER_PID"
}

# Function to run client
run_client() {
    echo "Starting client with options: $CMD_ARGS"
    "$PROJECT_ROOT/build/client" $CMD_ARGS &
    CLIENT_PID=$!
    echo $CLIENT_PID > /tmp/hologram_client.pid
    echo "Client started with PID: $CLIENT_PID"
}

# Run based on mode
case $MODE in
    "server")
        run_server
        ;;
    "client")
        run_client
        ;;
    "both")
        run_server
        sleep 2  # Wait for server to start
        run_client
        ;;
esac

# Wait for Ctrl+C
echo "Press Ctrl+C to stop the processes"
trap 'kill_existing_processes; exit 0' INT
wait
