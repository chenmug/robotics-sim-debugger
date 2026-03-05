#pragma once
#include "SimulationState.hpp" // Forward Declaration
#include "Robot.hpp"           // Forward Declaration
#include <vector>              // For std::vector
#include <memory>              // For std::unique_ptr

/**
 * @brief Configuration for the simulation grid.
 * 
 * Defines the size of the grid and positions of static obstacles.
 */
struct GridConfig 
{
    int width;                              // Width of the grid
    int height;                             // Height of the grid
    std::vector<Position> static_obstacles; // Positions of obstacles that cannot move
};

/**
 * @brief Core simulation engine that manages the simulation tick loop.
 * 
 * Responsible for running each simulation tick in the correct order: sense -> plan -> act.
 * 
 * The engine owns the SimulationState and the robots.
 */
class SimulationEngine 
{
private:
    SimulationState current_state;              // Current simulation state
    GridConfig grid;                            // Grid configuration
    std::vector<std::unique_ptr<Robot>> robots; // All robots in the simulation

public:
    /**
     * @brief Construct a new Simulation Engine.
     * 
     * Initializes the simulation state with the given grid configuration.
     * 
     * @param config Grid configuration (size, static obstacles)
     */
    SimulationEngine(const GridConfig& config);

    /**
     * @brief Add a robot to the simulation.
     * 
     * Robots are stored internally and will be updated in every tick.
     * 
     * @param robot Unique pointer to a Robot instance.
     */
    void addRobot(std::unique_ptr<Robot> robot);

    /**
     * @brief Advance the simulation by one tick.
     * 
     * Executes all robots in order: sense -> plan -> act.
     * Updates the internal SimulationState accordingly.
     */
    void runTick();

    /**
     * @brief Access the current simulation state.
     * 
     * Provides a const reference to the current SimulationState. Useful for rendering, 
     * logging, or snapshotting.
     * 
     * @return Current simulation state
     */
    const SimulationState& getCurrentState() const;
};