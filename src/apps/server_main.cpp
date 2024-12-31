#include "hologram/server/VolumetricServer.hpp"
#include <iostream>
#include <string>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <filesystem>

using namespace hologram;

std::atomic<bool> running(true);
const char* PID_FILE = "/tmp/hologram_server.pid";
bool verbose = true;  // Default to verbose logging

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]\n"
              << "Options:\n"
              << "  -c, --compression SCHEME  Set compression scheme (default: vivo)\n"
              << "  -s, --cell-size SIZE     Set cell size (default: 1.0)\n"
              << "  -q, --quiet              Disable verbose logging\n"
              << "  -h, --help               Show this help message\n";
}

void signalHandler(int signum) {
    if (verbose) {
        std::cout << "\nReceived signal " << signum << std::endl;
    }
    running = false;
}

void cleanup() {
    // Remove PID file on exit
    if (std::filesystem::exists(PID_FILE)) {
        std::filesystem::remove(PID_FILE);
        if (verbose) {
            std::cout << "Removed PID file" << std::endl;
        }
    }
}

void killExistingServer() {
    if (std::filesystem::exists(PID_FILE)) {
        std::ifstream pidFile(PID_FILE);
        std::string pidStr;
        if (pidFile >> pidStr) {
            if (verbose) {
                std::cout << "Found existing server process (PID: " << pidStr << "). Attempting to kill..." << std::endl;
            }
            
            // Try to kill the process
            std::string killCmd = "kill -9 " + pidStr + " 2>/dev/null || true";
            system(killCmd.c_str());
            
            // Wait a moment for the process to die
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Check if the process is still running
            std::string checkCmd = "kill -0 " + pidStr + " 2>/dev/null";
            if (system(checkCmd.c_str()) != 0) {
                if (verbose) {
                    std::cout << "Successfully killed existing server process" << std::endl;
                }
            } else {
                std::cerr << "Warning: Could not kill existing server process" << std::endl;
            }
        }
        std::filesystem::remove(PID_FILE);
    }
}

void writePidFile() {
    std::ofstream pidFile(PID_FILE);
    if (pidFile) {
        pidFile << getpid();
        pidFile.close();
        if (verbose) {
            std::cout << "Wrote PID file: " << getpid() << std::endl;
        }
    } else {
        std::cerr << "Warning: Could not write PID file" << std::endl;
    }
}

int main(int argc, char** argv) {
    try {
        std::string compressionScheme = "vivo";
        float cellSize = 1.0f;

        // Parse command line arguments
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "-h" || arg == "--help") {
                printUsage(argv[0]);
                return 0;
            } else if (arg == "-q" || arg == "--quiet") {
                verbose = false;
            } else if (arg == "-c" || arg == "--compression") {
                if (i + 1 < argc) {
                    compressionScheme = argv[++i];
                } else {
                    std::cerr << "Error: --compression requires a scheme" << std::endl;
                    return 1;
                }
            } else if (arg == "-s" || arg == "--cell-size") {
                if (i + 1 < argc) {
                    cellSize = std::stof(argv[++i]);
                } else {
                    std::cerr << "Error: --cell-size requires a value" << std::endl;
                    return 1;
                }
            }
        }

        // Kill any existing server process
        killExistingServer();

        // Register cleanup on exit
        std::atexit(cleanup);

        // Register signal handlers
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);

        // Write current PID to file
        writePidFile();

        if (verbose) {
            std::cout << "Starting server with:\n"
                      << "  Compression scheme: " << compressionScheme << "\n"
                      << "  Cell size: " << cellSize << "\n"
                      << "  Verbose logging: " << (verbose ? "enabled" : "disabled") << std::endl;
        }

        VolumetricServer server(compressionScheme, cellSize, verbose);
        server.start();

        if (verbose) {
            std::cout << "Server is running. Press Ctrl+C to stop..." << std::endl;
        }

        // Wait for signal
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (verbose) {
            std::cout << "Shutting down server..." << std::endl;
        }
        server.stop();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 