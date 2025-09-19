#pragma once
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>

/**
 * @brief Configuration system for the physics simulation
 * 
 * This class demonstrates:
 * - Configuration file parsing
 * - Default parameter handling
 * - Type-safe configuration access
 * - File I/O operations
 * - Error handling
 */
class SimulationConfig {
private:
    std::map<std::string, std::string> config_;
    
public:
    // Default configuration values
    struct Defaults {
        static constexpr int NUM_PARTICLES = 100;
        static constexpr int NUM_THREADS = 4;
        static constexpr double WORLD_WIDTH = 100.0;
        static constexpr double WORLD_HEIGHT = 50.0;
        static constexpr double GRAVITY = 0.5;
        static constexpr double DAMPING = 0.95;
        static constexpr double DT = 1.0/60.0;
        static constexpr int RENDER_WIDTH = 100;
        static constexpr int RENDER_HEIGHT = 50;
        static constexpr int TARGET_FPS = 60;
        static constexpr bool ENABLE_MULTITHREADING = true;
        static constexpr bool SHOW_STATS = true;
    };

    SimulationConfig() {
        // Set default values
        setDefaults();
    }

    // Load configuration from file
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Warning: Could not open config file: " << filename 
                      << ". Using defaults.\n";
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') continue;
            
            // Parse key=value pairs
            size_t pos = line.find('=');
            if (pos != std::string::npos) {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);
                
                // Trim whitespace
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);
                
                config_[key] = value;
            }
        }
        
        file.close();
        std::cout << "Loaded configuration from: " << filename << "\n";
        return true;
    }

    // Save configuration to file
    bool saveToFile(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not save config file: " << filename << "\n";
            return false;
        }

        file << "# Physics Simulation Configuration\n";
        file << "# Generated automatically\n\n";
        
        for (const auto& pair : config_) {
            file << pair.first << " = " << pair.second << "\n";
        }
        
        file.close();
        std::cout << "Saved configuration to: " << filename << "\n";
        return true;
    }

    // Type-safe getters with defaults
    int getInt(const std::string& key, int defaultValue = 0) const {
        auto it = config_.find(key);
        if (it != config_.end()) {
            try {
                return std::stoi(it->second);
            } catch (const std::exception&) {
                std::cerr << "Warning: Invalid integer value for " << key << ": " << it->second << "\n";
            }
        }
        return defaultValue;
    }

    double getDouble(const std::string& key, double defaultValue = 0.0) const {
        auto it = config_.find(key);
        if (it != config_.end()) {
            try {
                return std::stod(it->second);
            } catch (const std::exception&) {
                std::cerr << "Warning: Invalid double value for " << key << ": " << it->second << "\n";
            }
        }
        return defaultValue;
    }

    bool getBool(const std::string& key, bool defaultValue = false) const {
        auto it = config_.find(key);
        if (it != config_.end()) {
            std::string value = it->second;
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            return (value == "true" || value == "1" || value == "yes");
        }
        return defaultValue;
    }

    std::string getString(const std::string& key, const std::string& defaultValue = "") const {
        auto it = config_.find(key);
        return (it != config_.end()) ? it->second : defaultValue;
    }

    // Setters
    void set(const std::string& key, const std::string& value) {
        config_[key] = value;
    }

    void set(const std::string& key, int value) {
        config_[key] = std::to_string(value);
    }

    void set(const std::string& key, double value) {
        config_[key] = std::to_string(value);
    }

    void set(const std::string& key, bool value) {
        config_[key] = value ? "true" : "false";
    }

    // Print current configuration
    void print() const {
        std::cout << "\nCurrent Configuration:\n";
        std::cout << "====================\n";
        for (const auto& pair : config_) {
            std::cout << pair.first << " = " << pair.second << "\n";
        }
        std::cout << "\n";
    }

private:
    void setDefaults() {
        config_["num_particles"] = std::to_string(Defaults::NUM_PARTICLES);
        config_["num_threads"] = std::to_string(Defaults::NUM_THREADS);
        config_["world_width"] = std::to_string(Defaults::WORLD_WIDTH);
        config_["world_height"] = std::to_string(Defaults::WORLD_HEIGHT);
        config_["gravity"] = std::to_string(Defaults::GRAVITY);
        config_["damping"] = std::to_string(Defaults::DAMPING);
        config_["dt"] = std::to_string(Defaults::DT);
        config_["render_width"] = std::to_string(Defaults::RENDER_WIDTH);
        config_["render_height"] = std::to_string(Defaults::RENDER_HEIGHT);
        config_["target_fps"] = std::to_string(Defaults::TARGET_FPS);
        config_["enable_multithreading"] = "true";
        config_["show_stats"] = "true";
    }
};
