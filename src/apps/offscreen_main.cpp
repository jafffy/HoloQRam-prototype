#include "hologram/network/OffscreenClient.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

std::atomic<bool> running(true);

void signalHandler(int signum) {
    running = false;
}

int main(int argc, char* argv[]) {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    
    try {
        std::cout << "Starting HoloQRam Off-screen Client...\n";
        OffscreenClient client;
        
        while (running) {
            client.run();
        }
        
        std::cout << "\nShutting down...\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 