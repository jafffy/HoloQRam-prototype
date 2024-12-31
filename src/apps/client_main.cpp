#include "hologram/network/VolumetricClient.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    try {
        // Default server connection parameters
        std::string serverIP = "127.0.0.1";  // localhost
        int serverPort = 8765;               // default port

        // Parse command line arguments if provided
        if (argc > 1) {
            serverIP = argv[1];
        }
        if (argc > 2) {
            serverPort = std::stoi(argv[2]);
        }

        // Create and run the client
        hologram::VolumetricClient client(serverIP, serverPort);
        client.start();

        // Wait for user input to stop
        std::cout << "Press Enter to stop..." << std::endl;
        std::cin.get();

        client.stop();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 