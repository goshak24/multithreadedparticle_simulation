#pragma once
#include <cmath>

/**
 * @brief Represents a single particle in the physics simulation
 * 
 * This class demonstrates several C++ concepts:
 * - Encapsulation: Private data members with public interface
 * - Constructor initialization lists for efficiency
 * - Const correctness for methods that don't modify state
 * - Operator overloading for natural mathematical operations
 */
class Particle {
private:
    double x_, y_;           // Position (using double for better precision)
    double vx_, vy_;         // Velocity
    double mass_;            // Mass affects collision response
    double radius_;          // Collision radius
    bool active_;            // Whether particle is active in simulation

public:
    // Constructor with initialization list (more efficient than assignment)
    Particle(double x = 0.0, double y = 0.0, double vx = 0.0, double vy = 0.0, 
             double mass = 1.0, double radius = 0.5)
        : x_(x), y_(y), vx_(vx), vy_(vy), mass_(mass), radius_(radius), active_(true) {}

    // Getters (const methods - they don't modify the object)
    double getX() const { return x_; }
    double getY() const { return y_; }
    double getVX() const { return vx_; }
    double getVY() const { return vy_; }
    double getMass() const { return mass_; }
    double getRadius() const { return radius_; }
    bool isActive() const { return active_; }

    // Setters
    void setPosition(double x, double y) { x_ = x; y_ = y; }
    void setVelocity(double vx, double vy) { vx_ = vx; vy_ = vy; }
    void setActive(bool active) { active_ = active; }

    // Physics methods
    void update(double dt) {
        if (!active_) return;
        x_ += vx_ * dt;
        y_ += vy_ * dt;
    }

    void applyForce(double fx, double fy, double dt) {
        if (!active_) return;
        vx_ += (fx / mass_) * dt;
        vy_ += (fy / mass_) * dt;
    }

    // Collision detection with another particle
    bool collidesWith(const Particle& other) const {
        if (!active_ || !other.active_) return false;
        
        double dx = x_ - other.x_;
        double dy = y_ - other.y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        return distance < (radius_ + other.radius_);
    }

    // Calculate distance to another particle
    double distanceTo(const Particle& other) const {
        double dx = x_ - other.x_;
        double dy = y_ - other.y_;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Kinetic energy calculation
    double getKineticEnergy() const {
        return 0.5 * mass_ * (vx_ * vx_ + vy_ * vy_);
    }

    // Speed calculation
    double getSpeed() const {
        return std::sqrt(vx_ * vx_ + vy_ * vy_);
    }
};
