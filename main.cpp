#include <iostream>
#include <cstdlib>
#include <ctime>
#include "Simulation.h"

/**
 * @brief Main entry point for the Advanced Physics Simulation Engine
 * 
 * This demonstrates:
 * - Clean main function with minimal responsibilities
 * - Proper initialization and cleanup
 * - Exception handling
 * - Command line argument parsing (basic)
 */
int main(int argc, char* argv[]) {
    // Seed random number generator
    std::srand(static_cast<unsigned int>(std::time(nullptr))); 
    
    try {
        // Create and start simulation
        Simulation simulation;
        
        // Handle command line arguments (basic)
        if (argc > 1) {
            std::string arg = argv[1];
            if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: " << argv[0] << " [options]\n";
                std::cout << "Options:\n";
                std::cout << "  --help, -h     Show this help message\n";
                std::cout << "  --config FILE  Load configuration from FILE\n";
                return 0;
            } else if (arg == "--config" && argc > 2) {
                // Load custom config file
                std::cout << "Loading custom configuration from: " << argv[2] << "\n";
                // Note: In a full implementation, you'd pass this to Simulation
            }
        }
        
        // Start the simulation
        simulation.start();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred!" << std::endl;
        return 1;
    }
    
    return 0;
} 