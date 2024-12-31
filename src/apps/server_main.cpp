#include "hologram/server/VolumetricServer.hpp"
#include <iostream>
#include <string>

using namespace hologram;

int main(int argc, char** argv) {
    try {
        std::string compressionScheme = "vivo";
        float cellSize = 1.0f;

        if (argc > 1) {
            compressionScheme = argv[1];
        }
        if (argc > 2) {
            cellSize = std::stof(argv[2]);
        }

        VolumetricServer server(compressionScheme, cellSize);
        server.start();

        // Wait for user input to stop
        std::cout << "Press Enter to stop the server..." << std::endl;
        std::cin.get();

        server.stop();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 