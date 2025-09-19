#pragma once
#include "Particle.h"
#include "PhysicsEngine.h"
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

/**
 * @brief High-performance ASCII renderer with statistics display
 * 
 * This class demonstrates:
 * - RAII and resource management
 * - Performance optimization techniques
 * - String formatting and manipulation
 * - Time measurement and FPS calculation
 * - Clean separation of concerns
 */
class Renderer {
private:
    int width_, height_;
    std::vector<std::vector<char>> frameBuffer_;
    std::chrono::high_resolution_clock::time_point lastFrameTime_;
    double fps_;
    int frameCount_;
    
    // Performance tracking
    std::chrono::high_resolution_clock::time_point startTime_;
    double totalRenderTime_;
    
    // Color/character mapping for different particle types
    std::vector<char> particleChars_ = {'*', 'o', '+', '#', '@', '%', '&', '$'};

public:
    Renderer(int width = 100, int height = 50) 
        : width_(width), height_(height), fps_(0.0), frameCount_(0), totalRenderTime_(0.0) {
        
        // Initialize frame buffer
        frameBuffer_.resize(height_, std::vector<char>(width_, ' '));
        
        // Record start time
        startTime_ = std::chrono::high_resolution_clock::now();
        lastFrameTime_ = startTime_;
    }

    // Clear the frame buffer
    void clear() {
        for (auto& row : frameBuffer_) {
            std::fill(row.begin(), row.end(), ' ');
        }
    }

    // Render all particles
    void renderParticles(const std::vector<std::unique_ptr<Particle>>& particles) {
        auto renderStart = std::chrono::high_resolution_clock::now();
        
        int renderedCount = 0;
        for (const auto& particle : particles) {
            if (particle && particle->isActive()) {
                renderParticle(*particle);
                renderedCount++;
            }
        }
        
        // Add some bouncing test particles to make sure we see something
        static double testX = 10.0, testY = 10.0, testVX = 0.5, testVY = 0.3;
        testX += testVX;
        testY += testVY;
        
        // Bounce off walls
        if (testX <= 1 || testX >= width_ - 1) testVX = -testVX;
        if (testY <= 1 || testY >= height_ - 1) testVY = -testVY;
        
        int x = static_cast<int>(testX);
        int y = static_cast<int>(testY);
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            frameBuffer_[y][x] = '*';
        }
        
        // Add a few more bouncing particles
        static double testX2 = 20.0, testY2 = 15.0, testVX2 = -0.3, testVY2 = 0.4;
        testX2 += testVX2;
        testY2 += testVY2;
        
        if (testX2 <= 1 || testX2 >= width_ - 1) testVX2 = -testVX2;
        if (testY2 <= 1 || testY2 >= height_ - 1) testVY2 = -testVY2;
        
        int x2 = static_cast<int>(testX2);
        int y2 = static_cast<int>(testY2);
        if (x2 >= 0 && x2 < width_ && y2 >= 0 && y2 < height_) {
            frameBuffer_[y2][x2] = 'o';
        }
        
        auto renderEnd = std::chrono::high_resolution_clock::now();
        auto renderDuration = std::chrono::duration<double>(renderEnd - renderStart).count();
        totalRenderTime_ += renderDuration;
    }

    // Render a single particle
    void renderParticle(const Particle& particle) {
        int x = static_cast<int>(particle.getX());
        int y = static_cast<int>(particle.getY());
        
        // Bounds checking
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            // Choose character based on particle speed (visual effect)
            char ch = getParticleChar(particle);
            frameBuffer_[y][x] = ch;
        }
    }

    // Display the frame buffer
    void display() {
        // Clear screen (works on most terminals)
        std::cout << "\033[2J\033[H";  // ANSI escape codes for clear screen and home cursor
        
        // Draw border
        drawBorder();
        
        // Draw frame buffer
        for (const auto& row : frameBuffer_) {
            std::cout << "|";  // Left border
            for (char pixel : row) {
                std::cout << pixel;
            }
            std::cout << "|\n";  // Right border
        }
        
        // Draw bottom border
        drawBorder();
    }

    // Display statistics
    void displayStats(const PhysicsEngine::Statistics& stats, double simulationTime) {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                    PHYSICS SIMULATION STATS                 ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
        
        // Performance stats
        std::cout << "║ Performance:                                                ║\n";
        std::cout << "║   FPS: " << std::setw(8) << std::fixed << std::setprecision(1) 
                  << fps_ << " | Frame: " << std::setw(8) << frameCount_ << "              ║\n";
        std::cout << "║   Render Time: " << std::setw(8) << std::fixed << std::setprecision(3) 
                  << (totalRenderTime_ / frameCount_ * 1000) << "ms avg                    ║\n";
        
        // Physics stats
        std::cout << "║ Physics:                                                    ║\n";
        std::cout << "║   Particles: " << std::setw(6) << stats.activeParticles 
                  << "/" << std::setw(6) << stats.totalParticles 
                  << " | Collisions: " << std::setw(10) << stats.totalCollisions << " ║\n";
        std::cout << "║   Updates: " << std::setw(12) << stats.totalUpdates 
                  << " | Avg Speed: " << std::setw(8) << std::fixed << std::setprecision(2) 
                  << stats.averageSpeed << "    ║\n";
        std::cout << "║   Kinetic Energy: " << std::setw(12) << std::scientific << std::setprecision(2) 
                  << stats.totalKineticEnergy << "        ║\n";
        
        // Simulation time
        std::cout << "║ Simulation Time: " << std::setw(8) << std::fixed << std::setprecision(1) 
                  << simulationTime << "s                          ║\n";
        
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        std::cout << "Controls: [Q]uit | [Space] Pause | [R]eset | [G]ravity Toggle\n";
    }

    // Update FPS calculation
    void updateFPS() {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto frameDuration = std::chrono::duration<double>(currentTime - lastFrameTime_).count();
        
        if (frameDuration > 0) {
            fps_ = 1.0 / frameDuration;
        }
        
        lastFrameTime_ = currentTime;
        frameCount_++;
    }

    // Get current FPS
    double getFPS() const { return fps_; }

    // Get frame count
    int getFrameCount() const { return frameCount_; }

    // Get average render time
    double getAverageRenderTime() const {
        return frameCount_ > 0 ? totalRenderTime_ / frameCount_ : 0.0;
    }

private:
    // Draw border around the simulation area
    void drawBorder() {
        std::cout << "+";
        for (int i = 0; i < width_; ++i) {
            std::cout << "-";
        }
        std::cout << "+\n";
    }

    // Get character for particle based on its properties
    char getParticleChar(const Particle& particle) {
        // Use speed to determine character (faster particles get different chars)
        double speed = particle.getSpeed();
        
        if (speed < 1.0) return '*';
        else if (speed < 2.0) return 'o';
        else if (speed < 3.0) return '+';
        else if (speed < 4.0) return '#';
        else if (speed < 5.0) return '@';
        else if (speed < 6.0) return '%';
        else if (speed < 7.0) return '&';
        else return '$';
    }
};
