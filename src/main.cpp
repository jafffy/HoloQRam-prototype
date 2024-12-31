#include "VolumetricClient.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    try {
        VolumetricClient client;
        client.run();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 