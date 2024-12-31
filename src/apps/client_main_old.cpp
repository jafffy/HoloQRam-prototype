#include "network/OffscreenClient.hpp"
#include <iostream>
#include <string>

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]\n"
              << "Options:\n"
              << "  --compression <scheme>  Compression scheme to use (octree or rle, default: octree)\n"
              << "  --help                 Show this help message\n";
}

int main(int argc, char* argv[]) {
    std::string compressionScheme = "octree";
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--compression" && i + 1 < argc) {
            compressionScheme = argv[++i];
            if (compressionScheme != "octree" && compressionScheme != "rle") {
                std::cerr << "Error: Invalid compression scheme. Use 'octree' or 'rle'.\n";
                return 1;
            }
        } else {
            std::cerr << "Error: Unknown argument '" << arg << "'\n";
            printUsage(argv[0]);
            return 1;
        }
    }
    
    try {
        OffscreenClient client(compressionScheme);
        std::cout << "Using compression scheme: " << compressionScheme << std::endl;
        client.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 