#pragma once
#include "../Particle.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include <cstdint>

/**
 * @brief Manages multiple particles and their rendering
 * 
 * This class demonstrates:
 * - std::vector for dynamic arrays
 * - Smart pointers for memory management  
 * - Separation of concerns (logic vs rendering)
 * - Iterator patterns
 * - Range-based for loops
 */
class ParticleManager {
private:
    std::vector<std::unique_ptr<Particle>> particles_;  // Smart pointers for automatic cleanup
    std::vector<sf::CircleShape> circles_;              // SFML shapes for rendering
    int windowWidth_;
    int windowHeight_;
    double damping_;  // Energy loss on wall bounce

public:
    // Constructor
    ParticleManager(int windowWidth, int windowHeight, double damping = 0.9);
    
    // Destructor (automatically cleans up - RAII principle)
    ~ParticleManager() = default;
    
    // Add a single particle
    void addParticle(std::unique_ptr<Particle> particle);
    
    // Create and add a particle with parameters
    void createParticle(double x, double y, double vx, double vy, double mass = 1.0, double radius = 10.0);
    
    // Create random particle
    void createRandomParticle();
    
    // Create multiple random particles
    void createMultipleRandomParticles(int count);
    
    // Update all particles (physics)
    void update(float deltaTime);
    
    // Render all particles
    void render(sf::RenderWindow& window);
    
    // Get particle count
    size_t getParticleCount() const { return particles_.size(); }
    
    // Clear all particles
    void clear();
    
    // Get particles (const reference - read-only access)
    const std::vector<std::unique_ptr<Particle>>& getParticles() const { return particles_; }

private:
    // Handle wall collisions for a single particle
    void handleWallCollision(Particle& particle, size_t index);
    
    // Update visual representation
    void updateVisuals();
    
    // Create SFML shape for particle
    sf::CircleShape createCircleForParticle(const Particle& particle);
};