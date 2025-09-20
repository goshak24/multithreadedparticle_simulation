#include <SFML/Graphics.hpp>
#include <iostream>
#include "Particle.h"
#include <optional>

int main()
{
    std::cout << "Testing SFML 3.x with bouncing particle...\n";

    // SFML 3.x syntax - VideoMode constructor
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 600;
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(WINDOW_WIDTH, WINDOW_HEIGHT)), "Bouncing Particle");
    window.setFramerateLimit(60);

    // Create a particle
    Particle particle(
        100.0,    // x position
        100.0,    // y position
        150.0,    // x velocity (pixels per second)
        200.0,    // y velocity
        1.0,      // mass
        20.0      // radius
    );

    // Create SFML circle to render the particle
    sf::CircleShape circle(static_cast<float>(particle.getRadius()));
    circle.setFillColor(sf::Color::Green);
    circle.setOutlineThickness(2.0f);
    circle.setOutlineColor(sf::Color::White);
    // Set origin to center of circle for proper positioning
    circle.setOrigin(sf::Vector2f(circle.getRadius(), circle.getRadius()));

    // For timing
    sf::Clock clock;

    std::cout << "Particle created! Watch it bounce around.\n";
    std::cout << "Press ESC or close window to exit.\n";

    // Main game loop
    while (window.isOpen())
    {
        // Get delta time (time since last frame)
        float deltaTime = clock.restart().asSeconds();

        // SFML 3.x event handling - returns std::optional<Event>
        while (std::optional<sf::Event> event = window.pollEvent())
        {
            // Check event types using is<> template
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }

            // Check for ESC key press
            if (event->is<sf::Event::KeyPressed>())
            {
                const auto &keyEvent = event->getIf<sf::Event::KeyPressed>();
                if (keyEvent && keyEvent->code == sf::Keyboard::Key::Escape)
                {
                    window.close();
                }
            }
        }

        // Update particle physics
        particle.update(deltaTime);

        // Handle boundary collisions (bouncing)
        double x = particle.getX();
        double y = particle.getY();
        double vx = particle.getVX();
        double vy = particle.getVY();
        double radius = particle.getRadius();

        // Bounce off left/right walls
        if (x - radius < 0) {
            particle.setPosition(radius, y);
            particle.setVelocity(-vx * 0.9, vy);  // 0.9 = damping factor
            std::cout << "Bounced off left wall!\n";
        } 
        else if (x + radius > WINDOW_WIDTH) {
            particle.setPosition(WINDOW_WIDTH - radius, y);
            particle.setVelocity(-vx * 0.9, vy);
            std::cout << "Bounced off right wall!\n";
        }

        // Bounce off top/bottom walls
        if (y - radius < 0) {
            particle.setPosition(x, radius);
            particle.setVelocity(vx, -vy * 0.9);
            std::cout << "Bounced off top wall!\n";
        } 
        else if (y + radius > WINDOW_HEIGHT) {
            particle.setPosition(x, WINDOW_HEIGHT - radius);
            particle.setVelocity(vx, -vy * 0.9);
            std::cout << "Bounced off bottom wall!\n";
        }

        // Update circle position for rendering (SFML 3.x uses sf::Vector2f)
        circle.setPosition(sf::Vector2f(
            static_cast<float>(particle.getX()), 
            static_cast<float>(particle.getY())
        ));

        // Change color based on speed for visual feedback
        double speed = particle.getSpeed();
        std::uint8_t intensity = static_cast<std::uint8_t>(std::min(255.0, speed));
        circle.setFillColor(sf::Color(intensity, static_cast<std::uint8_t>(255 - intensity/2), 100, 200));

        // Render everything
        window.clear(sf::Color::Black);
        window.draw(circle);
        window.display();
    }

    std::cout << "Particle simulation completed!\n";
    return 0;
}