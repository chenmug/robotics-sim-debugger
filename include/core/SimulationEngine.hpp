#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include "core/SnapshotManager.hpp"  // Forward Declaration
#include "robots/Robot.hpp"          // Forward Declaration
#include <vector>                    // For std::vector
#include <memory>                    // For std::unique_ptr

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
    SimulationState current_state;               // Current simulation state
    GridConfig grid_;                            // Grid configuration
    std::vector<std::unique_ptr<Robot>> robots;  // All robots in the simulation
    SnapshotManager snapshotManager_;            // Stores snapshots for debugging / replay

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
     * @brief Add a robot to the simulation at a specific starting position.
     * 
     * This function registers the robot in the simulation engine, assigns it a unique ID,
     * initializes its state in the SimulationState, and stores ownership of the Robot instance.
     * 
     * @param robot Unique pointer to a Robot instance. The engine takes ownership of the robot.
     * @param start_pos Initial position of the robot in the grid. 
     * @param goal_pos End position of the robot in the grid. 
     */
    void addRobot(std::unique_ptr<Robot> robot, Position start_pos, Position goal_pos);

    /**
     * @brief Advance the simulation by one tick.
     * 
     * Executes all robots in order: sense -> plan -> act.
     * Updates the internal SimulationState accordingly.
     */
    void runTick();

    /**
     * @brief Replace the current simulation state.
     *
     * Used when restoring a snapshot during time-travel debugging.
     * 
     * @param state Current simulation state.
     */
    void setCurrentState(const SimulationState& state);

    /**
     * @brief Access the current simulation state.
     * 
     * Provides a const reference to the current SimulationState. Useful for rendering, 
     * logging, or snapshotting.
     * 
     * @return Current simulation state
     */
    const SimulationState& getCurrentState() const;

    /**
     * @brief Access the grid configuration.
     * 
     * Provides read-only access to the grid configuration used by the simulation.
     * Useful for rendering and debugging.
     * 
     * @return Grid configuration
     */
    const GridConfig& getGridConfig() const;

    /**
     * @brief Check if a position is within the grid boundaries.
     * @param pos The position to check.
     *
     * @return True if the position is in the grid boundaries, false otherwise.
     */
    bool isWithinBounds(const Position& pos) const;

    /**
     * @brief Checks if all robots reached their goals.
     * 
     * @return True if all robots reached their goals, false otherwise.
     */
    bool allRobotsReached() const;

    /**
     * @brief Get the number of robots currently registered in the simulation.
     * 
     * This function does not modify the engine or the simulation state.
     * 
     * @return Number of robots in the simulation.
     */
    size_t getRobotCount() const;

    /**
     * @brief Access the internal SnapshotManager.
     *
     * Provides access to the snapshot history for replaying past states or debugging purposes.
     *
     * @return reference to SnapshotManager
     */
    SnapshotManager& getSnapshotManager();

    /**
     * @brief Process all robot events and add the events in the current state.
     * 
     * This function adds events related to the robot's actions or environment during the 
     * current simulation tick.
     */
    void processEvents();

    /**
     * @brief Detects potential collisions between robots based on their planned next positions.
     *
     * This function checks for conflicts where two or more robots intend to move
     * into the same grid cell during the current simulation tick.
     *
     * The detection is performed after all robots have completed their planning phase
     * and before any movement (act phase) is executed, ensuring a consistent view
     * of all intended actions.
     */
    void detectCollisions();
};