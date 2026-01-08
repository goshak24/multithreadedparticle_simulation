#pragma once
#include "PhysicsEngine.h"
#include "Renderer.h"
#include "SimulationConfig.h"
#include "Particle.h"
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>

/**
 * @brief Main simulation class that orchestrates the entire physics simulation
 * 
 * This class demonstrates:
 * - Composition and dependency injection
 * - State management
 * - Event handling
 * - Performance monitoring
 * - Clean architecture principles
 */
class Simulation {
private:
    std::unique_ptr<PhysicsEngine> physicsEngine_;
    std::unique_ptr<Renderer> renderer_;
    SimulationConfig config_;
    
    // Simulation state
    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};
    std::chrono::high_resolution_clock::time_point startTime_;
    std::chrono::high_resolution_clock::time_point lastUpdateTime_;
    
    // Performance tracking
    double targetFrameTime_;
    int frameCount_;
    double totalSimulationTime_;

public:
    Simulation() : frameCount_(0), totalSimulationTime_(0.0) {
        // Load configuration
        config_.loadFromFile("simulation.conf");
        
        // Initialize components
        initializeComponents();
        
        // Calculate target frame time
        int targetFPS = config_.getInt("target_fps", SimulationConfig::Defaults::TARGET_FPS);
        targetFrameTime_ = 1.0 / targetFPS;
        
        std::cout << "Simulation initialized with " 
                  << config_.getInt("num_particles", SimulationConfig::Defaults::NUM_PARTICLES)
                  << " particles\n";
    }

    ~Simulation() {
        stop();
    }

    // Initialize all components
    void initializeComponents() {
        // Create physics engine
        physicsEngine_ = std::make_unique<PhysicsEngine>(
            config_.getInt("num_threads", SimulationConfig::Defaults::NUM_THREADS),
            config_.getDouble("world_width", SimulationConfig::Defaults::WORLD_WIDTH),
            config_.getDouble("world_height", SimulationConfig::Defaults::WORLD_HEIGHT),
            config_.getDouble("gravity", SimulationConfig::Defaults::GRAVITY),
            config_.getDouble("damping", SimulationConfig::Defaults::DAMPING)
        );

        // Create renderer
        renderer_ = std::make_unique<Renderer>(
            config_.getInt("render_width", SimulationConfig::Defaults::RENDER_WIDTH),
            config_.getInt("render_height", SimulationConfig::Defaults::RENDER_HEIGHT)
        );

        // Add particles
        int numParticles = config_.getInt("num_particles", SimulationConfig::Defaults::NUM_PARTICLES);
        for (int i = 0; i < numParticles; ++i) {
            physicsEngine_->addRandomParticle();
        }
        
        // Add some fixed particles for testing
        auto particle1 = std::make_unique<Particle>(10.0, 10.0, 1.0, 1.0, 1.0, 0.5);  // Faster velocities
        auto particle2 = std::make_unique<Particle>(20.0, 15.0, -1.0, 1.0, 1.0, 0.5);
        auto particle3 = std::make_unique<Particle>(30.0, 20.0, 1.0, -1.0, 1.0, 0.5);
        auto particle4 = std::make_unique<Particle>(40.0, 25.0, -1.0, -1.0, 1.0, 0.5);
        auto particle5 = std::make_unique<Particle>(50.0, 30.0, 0.0, 1.0, 1.0, 0.5);
        
        physicsEngine_->addParticle(std::move(particle1));
        physicsEngine_->addParticle(std::move(particle2));
        physicsEngine_->addParticle(std::move(particle3));
        physicsEngine_->addParticle(std::move(particle4));
        physicsEngine_->addParticle(std::move(particle5));
    }

    // Start the simulation
    void start() {
        if (running_) return;
        
        running_ = true;
        startTime_ = std::chrono::high_resolution_clock::now();
        lastUpdateTime_ = startTime_;
        
        std::cout << "Starting simulation...\n";
        std::cout << "Controls: [Q]uit | [Space] Pause | [R]eset | [G]ravity Toggle\n";
        
        // Start physics engine if multithreading is enabled
        if (config_.getBool("enable_multithreading", true)) {
            physicsEngine_->start();
        }
        
        // Main simulation loop
        runSimulationLoop();
    }

    // Stop the simulation
    void stop() {
        if (!running_) return;
        
        running_ = false;
        physicsEngine_->stop();
        
        auto endTime = std::chrono::high_resolution_clock::now();
        auto totalDuration = std::chrono::duration<double>(endTime - startTime_).count();
        
        std::cout << "\nSimulation stopped.\n";
        std::cout << "Total runtime: " << totalDuration << " seconds\n";
        std::cout << "Total frames: " << frameCount_ << "\n";
        std::cout << "Average FPS: " << (frameCount_ / totalDuration) << "\n";
    }

    // Toggle pause state
    void togglePause() {
        paused_ = !paused_;
        std::cout << (paused_ ? "Simulation paused" : "Simulation resumed") << "\n";
    }

    // Reset simulation
    void reset() {
        std::cout << "Resetting simulation...\n";
        
        // Stop current simulation
        physicsEngine_->stop();
        
        // Reinitialize components
        initializeComponents();
        
        // Restart if was running
        if (running_) {
            physicsEngine_->start();
        }
        
        // Reset timing
        startTime_ = std::chrono::high_resolution_clock::now();
        lastUpdateTime_ = startTime_;
        frameCount_ = 0;
        totalSimulationTime_ = 0.0;
    }

    // Toggle gravity
    void toggleGravity() {
        double currentGravity = config_.getDouble("gravity", SimulationConfig::Defaults::GRAVITY);
        double newGravity = (currentGravity == 0.0) ? 0.5 : 0.0;
        
        config_.set("gravity", newGravity);
        physicsEngine_->setGravity(newGravity);
        
        std::cout << "Gravity " << (newGravity == 0.0 ? "disabled" : "enabled") << "\n";
    }

    // Handle user input
    void handleInput() {
        // Check if user wants to quit (Ctrl+C or 'q' key)
        // For now, we'll add a simple timeout mechanism
        static int frameCount = 0;
        frameCount++;
        
        // Auto-quit after 10 seconds to prevent infinite running
        if (frameCount > 600) {  // ~10 seconds at 60 FPS
            std::cout << "\nSimulation completed after 10 seconds.\n";
            running_ = false;
        }
    }

    // Get simulation statistics
    PhysicsEngine::Statistics getStatistics() const {
        return physicsEngine_->getStatistics();
    }

    // Get configuration
    const SimulationConfig& getConfig() const {
        return config_;
    }

private:
    // Main simulation loop
    void runSimulationLoop() {
        while (running_) {
            auto frameStart = std::chrono::high_resolution_clock::now();
            
            // Handle input
            handleInput();
            
            // Update physics if not paused
            if (!paused_) {
                updatePhysics();
            }
            
            // Render frame
            renderFrame();
            
            // Calculate frame time and sleep if necessary
            auto frameEnd = std::chrono::high_resolution_clock::now();
            auto frameDuration = std::chrono::duration<double>(frameEnd - frameStart).count();
            
            // Sleep to maintain target FPS
            if (frameDuration < targetFrameTime_) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(targetFrameTime_ - frameDuration)
                );
            }
            
            frameCount_++;
            
            // Update simulation time
            auto currentTime = std::chrono::high_resolution_clock::now();
            totalSimulationTime_ = std::chrono::duration<double>(currentTime - startTime_).count();
            
            // Auto-quit after 15 seconds to prevent infinite running
            if (totalSimulationTime_ > 15.0) {
                std::cout << "\nSimulation completed after 15 seconds.\n";
                std::cout << "Press Ctrl+C to quit if still running.\n";
                running_ = false;
                break;
            }
        }
    }

    // Update physics
    void updatePhysics() {
        double dt = config_.getDouble("dt", SimulationConfig::Defaults::DT);
        
        if (config_.getBool("enable_multithreading", true)) {
            // Physics engine handles its own threading
            // Just update timing
            lastUpdateTime_ = std::chrono::high_resolution_clock::now();
        } else {
            // Single-threaded update
            physicsEngine_->update(dt);
            lastUpdateTime_ = std::chrono::high_resolution_clock::now();
        }
    }

    // Render a frame
    void renderFrame() {
        // Clear renderer
        renderer_->clear();
        
        // Render particles
        renderer_->renderParticles(physicsEngine_->getParticles());
        
        // Update FPS
        renderer_->updateFPS();
        
        // Display frame
        renderer_->display();
        
        // Display statistics if enabled
        if (config_.getBool("show_stats", true)) {
            auto stats = physicsEngine_->getStatistics();
            renderer_->displayStats(stats, totalSimulationTime_);
        }
    }
};
