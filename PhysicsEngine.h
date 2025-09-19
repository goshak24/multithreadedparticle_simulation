#pragma once
#include "Particle.h"
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <functional>

/**
 * @brief Advanced physics engine with multithreading and collision detection
 * 
 * This class demonstrates advanced C++ concepts:
 * - RAII (Resource Acquisition Is Initialization)
 * - Smart pointers for memory management
 * - Thread safety with mutexes and atomics
 * - Lambda functions and std::function
 * - Template programming concepts
 * - Exception safety
 */
class PhysicsEngine {
private:
    std::vector<std::unique_ptr<Particle>> particles_;  // Smart pointers for automatic memory management
    std::vector<std::thread> workerThreads_;            // Thread pool
    std::mutex particlesMutex_;                         // Thread synchronization
    std::atomic<bool> running_{false};                  // Atomic for thread-safe flag
    std::atomic<int> activeParticles_{0};               // Thread-safe counter
    
    // Configuration
    int numThreads_;
    double worldWidth_, worldHeight_;
    double gravity_;
    double damping_;  // Energy loss on collisions
    
    // Performance monitoring
    std::atomic<long long> totalCollisions_{0};
    std::atomic<long long> totalUpdates_{0};

public:
    // Constructor with default parameters
    PhysicsEngine(int numThreads = std::thread::hardware_concurrency(),
                  double worldWidth = 100.0, double worldHeight = 50.0,
                  double gravity = 0.0, double damping = 0.95)
        : numThreads_(numThreads), worldWidth_(worldWidth), worldHeight_(worldHeight),
          gravity_(gravity), damping_(damping) {}

    // Destructor - automatically stops threads (RAII)
    ~PhysicsEngine() {
        stop();
    }

    // Add particle with move semantics for efficiency
    void addParticle(std::unique_ptr<Particle> particle) {
        std::lock_guard<std::mutex> lock(particlesMutex_);
        particles_.push_back(std::move(particle));
        activeParticles_++;
    }

    // Create and add a random particle
    void addRandomParticle() {
        auto particle = std::make_unique<Particle>(
            static_cast<double>(rand() % static_cast<int>(worldWidth_)),
            static_cast<double>(rand() % static_cast<int>(worldHeight_)),
            (rand() % 6 - 3) / 2.0,  // Small random velocity (-1.5 to 1.5)
            (rand() % 6 - 3) / 2.0,
            1.0 + (rand() % 10) / 10.0,  // Random mass
            0.5 + (rand() % 5) / 10.0    // Random radius
        );
        addParticle(std::move(particle));
    }

    // Start the physics simulation
    void start() {
        if (running_) return;
        
        running_ = true;
        workerThreads_.clear();
        
        // Create worker threads
        for (int i = 0; i < numThreads_; ++i) {
            workerThreads_.emplace_back([this, i]() {
                this->workerThread(i);
            });
        }
    }

    // Stop the physics simulation
    void stop() {
        running_ = false;
        
        // Wait for all threads to finish
        for (auto& thread : workerThreads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        workerThreads_.clear();
    }

    // Update physics for one frame
    void update(double dt) {
        if (!running_) {
            // Single-threaded update for testing
            updateParticles(dt);
            handleCollisions();
            handleBoundaries();
        }
    }

    // Get particles for rendering (const reference to avoid copying)
    const std::vector<std::unique_ptr<Particle>>& getParticles() {
        return particles_;
    }

    // Get statistics
    struct Statistics {
        int totalParticles;
        int activeParticles;
        long long totalCollisions;
        long long totalUpdates;
        double averageSpeed;
        double totalKineticEnergy;
    };

    Statistics getStatistics() {
        Statistics stats{};
        stats.totalParticles = particles_.size();
        stats.activeParticles = activeParticles_.load();
        stats.totalCollisions = totalCollisions_.load();
        stats.totalUpdates = totalUpdates_.load();
        
        double totalSpeed = 0.0;
        double totalKE = 0.0;
        int count = 0;
        
        std::lock_guard<std::mutex> lock(particlesMutex_);
        for (const auto& particle : particles_) {
            if (particle && particle->isActive()) {
                totalSpeed += particle->getSpeed();
                totalKE += particle->getKineticEnergy();
                count++;
            }
        }
        
        stats.averageSpeed = count > 0 ? totalSpeed / count : 0.0;
        stats.totalKineticEnergy = totalKE;
        
        return stats;
    }

    // Configuration methods
    void setGravity(double gravity) { gravity_ = gravity; }
    void setDamping(double damping) { damping_ = damping; }
    void setWorldSize(double width, double height) { 
        worldWidth_ = width; 
        worldHeight_ = height; 
    }

private:
    // Worker thread function
    void workerThread(int threadId) {
        while (running_) {
            // Each thread processes a portion of particles
            int particlesPerThread = particles_.size() / numThreads_;
            int start = threadId * particlesPerThread;
            int end = (threadId == numThreads_ - 1) ? particles_.size() : start + particlesPerThread;
            
            updateParticleRange(start, end, 1.0/60.0);  // 60 FPS
            
            // Small delay to prevent excessive CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // Update particles in a specific range
    void updateParticleRange(int start, int end, double dt) {
        std::lock_guard<std::mutex> lock(particlesMutex_);
        
        for (int i = start; i < end && i < particles_.size(); ++i) {
            if (particles_[i] && particles_[i]->isActive()) {
                // Apply gravity
                particles_[i]->applyForce(0, gravity_ * particles_[i]->getMass(), dt);
                
                // Update position
                particles_[i]->update(dt);
                
                totalUpdates_++;
            }
        }
    }

    // Update all particles (single-threaded version)
    void updateParticles(double dt) {
        std::lock_guard<std::mutex> lock(particlesMutex_);
        
        for (auto& particle : particles_) {
            if (particle && particle->isActive()) {
                // Apply gravity
                particle->applyForce(0, gravity_ * particle->getMass(), dt);
                
                // Update position
                particle->update(dt);
                
                totalUpdates_++;
            }
        }
        
        // Handle boundaries after updating positions (without locking again)
        for (auto& particle : particles_) {
            if (!particle || !particle->isActive()) continue;
            
            double x = particle->getX();
            double y = particle->getY();
            double vx = particle->getVX();
            double vy = particle->getVY();
            double radius = particle->getRadius();
            
            // Bounce off walls with energy loss
            if (x - radius < 0) {
                particle->setVelocity(-vx * damping_, vy);
                particle->setPosition(radius, y);
            } else if (x + radius > worldWidth_) {
                particle->setVelocity(-vx * damping_, vy);
                particle->setPosition(worldWidth_ - radius, y);
            }
            
            if (y - radius < 0) {
                particle->setVelocity(vx, -vy * damping_);
                particle->setPosition(x, radius);
            } else if (y + radius > worldHeight_) {
                particle->setVelocity(vx, -vy * damping_);
                particle->setPosition(x, worldHeight_ - radius);
            }
        }
    }

    // Handle particle-particle collisions
    void handleCollisions() {
        std::lock_guard<std::mutex> lock(particlesMutex_);
        
        for (size_t i = 0; i < particles_.size(); ++i) {
            if (!particles_[i] || !particles_[i]->isActive()) continue;
            
            for (size_t j = i + 1; j < particles_.size(); ++j) {
                if (!particles_[j] || !particles_[j]->isActive()) continue;
                
                if (particles_[i]->collidesWith(*particles_[j])) {
                    resolveCollision(*particles_[i], *particles_[j]);
                    totalCollisions_++;
                }
            }
        }
    }

    // Resolve collision between two particles (elastic collision)
    void resolveCollision(Particle& p1, Particle& p2) {
        // Calculate collision normal
        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance == 0) return;  // Avoid division by zero
        
        double nx = dx / distance;
        double ny = dy / distance;
        
        // Relative velocity
        double dvx = p2.getVX() - p1.getVX();
        double dvy = p2.getVY() - p1.getVY();
        
        // Relative velocity along collision normal
        double dvn = dvx * nx + dvy * ny;
        
        // Don't resolve if velocities are separating
        if (dvn > 0) return;
        
        // Collision impulse
        double impulse = 2 * dvn / (p1.getMass() + p2.getMass());
        
        // Update velocities
        p1.setVelocity(p1.getVX() + impulse * p2.getMass() * nx * damping_,
                      p1.getVY() + impulse * p2.getMass() * ny * damping_);
        p2.setVelocity(p2.getVX() - impulse * p1.getMass() * nx * damping_,
                      p2.getVY() - impulse * p1.getMass() * ny * damping_);
    }

    // Handle boundary collisions
    void handleBoundaries() {
        std::lock_guard<std::mutex> lock(particlesMutex_);
        
        for (auto& particle : particles_) {
            if (!particle || !particle->isActive()) continue;
            
            double x = particle->getX();
            double y = particle->getY();
            double vx = particle->getVX();
            double vy = particle->getVY();
            double radius = particle->getRadius();
            
            // Bounce off walls with energy loss
            if (x - radius < 0) {
                particle->setVelocity(-vx * damping_, vy);
                particle->setPosition(radius, y);
            } else if (x + radius > worldWidth_) {
                particle->setVelocity(-vx * damping_, vy);
                particle->setPosition(worldWidth_ - radius, y);
            }
            
            if (y - radius < 0) {
                particle->setVelocity(vx, -vy * damping_);
                particle->setPosition(x, radius);
            } else if (y + radius > worldHeight_) {
                particle->setVelocity(vx, -vy * damping_);
                particle->setPosition(x, worldHeight_ - radius);
            }
        }
    }
};
