#include <iostream>
#include <csignal>
#include <atomic>
#include <string>

// Boost includes (before PCL)
#include <boost/concept_check.hpp>

// Project includes
#include "hologram/network/OffscreenClient.hpp"
#include "hologram/compression/ViVoCompression.hpp"

using namespace hologram;

std::atomic<bool> running(true);

void signalHandler(int signum) {
    running = false;
}

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]\n"
              << "Options:\n"
              << "  --compression <scheme>  Compression scheme to use (vivo, octree, rle)\n"
              << "  --server <ip>          Server IP address (default: 127.0.0.1)\n"
              << "  --port <port>          Server port (default: 8765)\n"
              << "  --help                 Show this help message\n";
}

int main(int argc, char* argv[]) {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    
    try {
        // Default configuration
        std::string compressionScheme = "vivo";
        std::string serverIP = "127.0.0.1";
        int serverPort = 8765;
        
        // Parse command line arguments
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            
            if (arg == "--help") {
                printUsage(argv[0]);
                return 0;
            } else if (arg == "--compression" && i + 1 < argc) {
                compressionScheme = argv[++i];
            } else if (arg == "--server" && i + 1 < argc) {
                serverIP = argv[++i];
            } else if (arg == "--port" && i + 1 < argc) {
                serverPort = std::stoi(argv[++i]);
            } else {
                std::cerr << "Unknown argument: " << arg << "\n";
                printUsage(argv[0]);
                return 1;
            }
        }
        
        std::cout << "Starting HoloQRam Off-screen Client...\n"
                  << "Compression: " << compressionScheme << "\n"
                  << "Server: " << serverIP << ":" << serverPort << "\n\n";
        
        // Create client with configuration
        OffscreenClient client(compressionScheme, serverIP, serverPort);
        
        try {
            client.run();
        } catch (const std::exception& e) {
            std::cerr << "Error during client run: " << e.what() << std::endl;
        }
        
        std::cout << "\nShutting down...\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 