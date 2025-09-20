#include <SFML/Graphics.hpp>
#include <iostream>
#include <cstdint>
#include <optional>
#include "ParticleManager.h" 

int main()
{
    std::cout << "Starting Multi-Particle Physics Simulation...\n";

    // Window constants
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 600;
    
    // Create SFML window
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(WINDOW_WIDTH, WINDOW_HEIGHT)), 
                           "Multi-Particle Physics Simulation");
    window.setFramerateLimit(60);

    // Create particle manager
    ParticleManager particleManager(WINDOW_WIDTH, WINDOW_HEIGHT, 0.9);  // 0.9 = damping factor

    // Create some initial particles
    std::cout << "Creating initial particles...\n";
    
    // Method 1: Create specific particles
    particleManager.createParticle(100, 100, 150, 200, 1.0, 15.0);  // Large fast particle
    particleManager.createParticle(300, 200, -100, 150, 0.5, 8.0);  // Small fast particle
    particleManager.createParticle(500, 300, 75, -120, 1.5, 20.0);  // Heavy slow particle
    
    // Method 2: Create random particles
    particleManager.createMultipleRandomParticles(7);  // Add 7 random particles
    
    std::cout << "Total particles: " << particleManager.getParticleCount() << "\n";
    
    // Game state
    bool showInstructions = true;
    sf::Clock clock;
    
    // Display controls
    std::cout << "\n=== CONTROLS ===\n";
    std::cout << "ESC - Quit\n";
    std::cout << "R - Add random particle\n";
    std::cout << "C - Clear all particles\n";
    std::cout << "SPACE - Add 5 random particles\n";
    std::cout << "================\n\n";

    // Main game loop
    while (window.isOpen())
    {
        float deltaTime = clock.restart().asSeconds();

        // Event handling
        while (std::optional<sf::Event> event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }

            if (event->is<sf::Event::KeyPressed>())
            {
                const auto& keyEvent = event->getIf<sf::Event::KeyPressed>();
                if (keyEvent) {
                    switch (keyEvent->code) {
                        case sf::Keyboard::Key::Escape:
                            std::cout << "Exiting simulation...\n";
                            window.close();
                            break;
                            
                        case sf::Keyboard::Key::R:
                            particleManager.createRandomParticle();
                            std::cout << "Added random particle. Total: " 
                                     << particleManager.getParticleCount() << "\n";
                            break;
                            
                        case sf::Keyboard::Key::C:
                            particleManager.clear();
                            std::cout << "Cleared all particles\n";
                            break;
                            
                        case sf::Keyboard::Key::Space:
                            particleManager.createMultipleRandomParticles(5);
                            std::cout << "Added 5 particles. Total: " 
                                     << particleManager.getParticleCount() << "\n";
                            break;
                            
                        default:
                            // Do nothing for other keys
                            break;
                    }
                }
            }
        }

        // Update physics
        particleManager.update(deltaTime);

        // Render everything
        window.clear(sf::Color::Black);
        particleManager.render(window);
        window.display();

        // Optional: Print particle count every few seconds (for debugging)
        static sf::Clock printClock;
        if (printClock.getElapsedTime().asSeconds() > 5.0f) {
            std::cout << "Active particles: " << particleManager.getParticleCount() << "\n";
            printClock.restart();
        }
    }

    std::cout << "Simulation ended. Final particle count: " 
              << particleManager.getParticleCount() << "\n";
    return 0;
}