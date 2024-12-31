#include "hologram/network/VolumetricClient.hpp"
#include <iostream>
#include <string>

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]\n"
              << "Options:\n"
              << "  -c, --compression <scheme>  Compression scheme to use (vivo, octree, rle)\n"
              << "  -s, --cell-size <size>     Cell size for compression\n"
              << "  -q, --quiet                Quiet mode\n"
              << "  --server <ip>              Server IP address (default: 127.0.0.1)\n"
              << "  --port <port>              Server port (default: 8765)\n"
              << "  -h, --help                 Show this help message\n";
}

int main(int argc, char* argv[]) {
    try {
        // Default parameters
        std::string serverIP = "127.0.0.1";  // localhost
        int serverPort = 8765;               // default port
        std::string compressionScheme = "vivo";
        double cellSize = 1.0;
        bool verbose = true;
        
        // Parse command line arguments
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            
            if (arg == "-h" || arg == "--help") {
                printUsage(argv[0]);
                return 0;
            } else if ((arg == "--server") && i + 1 < argc) {
                serverIP = argv[++i];
            } else if ((arg == "--port") && i + 1 < argc) {
                serverPort = std::stoi(argv[++i]);
            } else if ((arg == "-c" || arg == "--compression") && i + 1 < argc) {
                compressionScheme = argv[++i];
            } else if ((arg == "-s" || arg == "--cell-size") && i + 1 < argc) {
                try {
                    cellSize = std::stod(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid cell size value: " << argv[i] << std::endl;
                    return 1;
                }
            } else if (arg == "-q" || arg == "--quiet") {
                verbose = false;
            } else {
                std::cerr << "Unknown argument: " << arg << std::endl;
                printUsage(argv[0]);
                return 1;
            }
        }

        if (verbose) {
            std::cout << "Starting client with:\n"
                      << "  Server: " << serverIP << ":" << serverPort << "\n"
                      << "  Compression: " << compressionScheme << "\n"
                      << "  Cell size: " << cellSize << std::endl;
        }

        // Create and run the client
        hologram::VolumetricClient client(serverIP, serverPort);
        client.start();

        // Wait for user input to stop
        if (verbose) {
            std::cout << "Press Enter to stop..." << std::endl;
        }
        std::cin.get();

        client.stop();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 