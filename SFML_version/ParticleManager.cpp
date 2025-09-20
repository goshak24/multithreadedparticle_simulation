#include "ParticleManager.h"
#include <random>
#include <iostream>
#include <algorithm>

// Constructor - Initialize member variables
ParticleManager::ParticleManager(int windowWidth, int windowHeight, double damping)
    : windowWidth_(windowWidth), windowHeight_(windowHeight), damping_(damping) {
    // Reserve space to avoid frequent reallocations
    particles_.reserve(100);  // Expect up to 100 particles
    circles_.reserve(100);
}

// Add a particle using move semantics (efficient transfer of ownership)
void ParticleManager::addParticle(std::unique_ptr<Particle> particle) {
    if (!particle) {
        std::cerr << "Warning: Attempted to add null particle\n";
        return;
    }
    
    // Create visual representation
    sf::CircleShape circle = createCircleForParticle(*particle);
    
    // Move particle into our container (transfers ownership)
    particles_.push_back(std::move(particle));
    circles_.push_back(std::move(circle));
    
    std::cout << "Added particle. Total count: " << particles_.size() << "\n";
}

// Create and add a particle with specific parameters
void ParticleManager::createParticle(double x, double y, double vx, double vy, double mass, double radius) {
    // Create unique_ptr using make_unique (C++14 feature)
    auto particle = std::make_unique<Particle>(x, y, vx, vy, mass, radius);
    addParticle(std::move(particle));
}

// Create a random particle
void ParticleManager::createRandomParticle() {
    // Static variables preserve state between function calls
    static std::random_device rd;
    static std::mt19937 gen(rd());  // Mersenne Twister generator
    
    // Distribution objects for random values
    std::uniform_real_distribution<double> posX(50.0, windowWidth_ - 50.0);
    std::uniform_real_distribution<double> posY(50.0, windowHeight_ - 50.0);
    std::uniform_real_distribution<double> vel(-200.0, 200.0);
    std::uniform_real_distribution<double> mass(0.5, 2.0);
    std::uniform_real_distribution<double> radius(5.0, 20.0);
    
    createParticle(posX(gen), posY(gen), vel(gen), vel(gen), mass(gen), radius(gen));
}

// Create multiple random particles
void ParticleManager::createMultipleRandomParticles(int count) {
    for (int i = 0; i < count; ++i) {
        createRandomParticle();
    }
    std::cout << "Created " << count << " random particles\n";
}

// Update all particles - this is where the physics happens
void ParticleManager::update(float deltaTime) {
    // Range-based for loop (C++11 feature) - cleaner than traditional for loops
    for (size_t i = 0; i < particles_.size(); ++i) {
        if (particles_[i] && particles_[i]->isActive()) {
            // Update physics
            particles_[i]->update(deltaTime);
            
            // Handle wall collisions
            handleWallCollision(*particles_[i], i);
        }
    }
    
    // Update visual representation
    updateVisuals();
}

// Render all particles to the window
void ParticleManager::render(sf::RenderWindow& window) {
    // Range-based for loop with const reference (read-only access)
    for (const auto& circle : circles_) {
        window.draw(circle);
    }
}

// Clear all particles
void ParticleManager::clear() {
    particles_.clear();  // Smart pointers automatically clean up memory
    circles_.clear();
    std::cout << "Cleared all particles\n";
}

// Handle wall collision for a single particle
void ParticleManager::handleWallCollision(Particle& particle, size_t index) {
    double x = particle.getX();
    double y = particle.getY();
    double vx = particle.getVX();
    double vy = particle.getVY();
    double radius = particle.getRadius();
    
    bool bounced = false;
    
    // Left/right wall collisions
    if (x - radius < 0) {
        particle.setPosition(radius, y);
        particle.setVelocity(-vx * damping_, vy);
        bounced = true;
    } else if (x + radius > windowWidth_) {
        particle.setPosition(windowWidth_ - radius, y);
        particle.setVelocity(-vx * damping_, vy);
        bounced = true;
    }
    
    // Top/bottom wall collisions
    if (y - radius < 0) {
        particle.setPosition(particle.getX(), radius);
        particle.setVelocity(particle.getVX(), -vy * damping_);
        bounced = true;
    } else if (y + radius > windowHeight_) {
        particle.setPosition(particle.getX(), windowHeight_ - radius);
        particle.setVelocity(particle.getVX(), -vy * damping_);
        bounced = true;
    }
    
    // Optional: Print bounce info (remove this for better performance with many particles)
    if (bounced && particles_.size() < 10) {  // Only print for small numbers of particles
        std::cout << "Particle " << index << " bounced! Speed: " 
                  << static_cast<int>(particle.getSpeed()) << "\n";
    }
}

// Update visual representation of all particles
void ParticleManager::updateVisuals() {
    // Update each circle to match its corresponding particle
    for (size_t i = 0; i < particles_.size() && i < circles_.size(); ++i) {
        if (particles_[i] && particles_[i]->isActive()) {
            // Update position
            circles_[i].setPosition(sf::Vector2f(
                static_cast<float>(particles_[i]->getX()),
                static_cast<float>(particles_[i]->getY())
            ));
            
            // Update color based on speed
            double speed = particles_[i]->getSpeed();
            std::uint8_t intensity = static_cast<std::uint8_t>(std::min(255.0, speed));
            std::uint8_t green = static_cast<std::uint8_t>(255 - intensity / 2);
            circles_[i].setFillColor(sf::Color(intensity, green, 100, 200));
        }
    }
}

// Create SFML circle for a particle
sf::CircleShape ParticleManager::createCircleForParticle(const Particle& particle) {
    sf::CircleShape circle(static_cast<float>(particle.getRadius()));
    
    // Set visual properties
    circle.setFillColor(sf::Color::Green);
    circle.setOutlineThickness(1.0f);
    circle.setOutlineColor(sf::Color::White);
    
    // Set origin to center for proper positioning
    circle.setOrigin(sf::Vector2f(circle.getRadius(), circle.getRadius()));
    
    // Set initial position
    circle.setPosition(sf::Vector2f(
        static_cast<float>(particle.getX()),
        static_cast<float>(particle.getY())
    ));
    
    return circle;
}