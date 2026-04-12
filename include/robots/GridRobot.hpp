#pragma once
#include "robots/Robot.hpp"           // Forward Declaration
#include "core/SimulationEngine.hpp"  // For GridConfig
#include <memory>                     // For std::shared_ptr


/**
 * @brief A simple robot that navigates on a 2D grid towards a goal.
 * 
 * Implements the Robot interface for the MVP simulation:
 * - Manages the robot's state in the simulation (current position, goal, next step).
 * - Delegates path planning, collision avoidance, and boundary checks to the assigned Planner.
 * - Updates its planned path and next move per simulation tick based on the Planner's output.
 * - Stores a local cache of current position, goal, and next step for efficiency.
 */
class GridRobot : public Robot
{
private:
    // Grid dimensions
    GridConfig grid_;  // Grid configuration (size + static obstacles)
    
    // Cache for robot convenience (always sync with SimulationState at start of tick)
    Position currentPos_{0,0};  // Current position (cached from SimulationState).
    Position goal_{0,0};        // Goal position (cached from SimulationState).
    Position nextPos_{0,0};     // Next position to move to.  
    
    // Cache for planning
    std::vector<Position> planned_path_cache_;  // Cached planned path for the robot.
    size_t path_index_cache_ = 0;               // Cached index of the current position in the planned path.

    // Planner strategy
    std::shared_ptr<Planner> planner_;  // Planner strategy used by this robot.

public:

    /**
     * @brief Construct a GridRobot with grid size and optional planner.
     * @param grid Grid configuration.
     * @param planner Optional shared pointer to a Planner strategy.
     */
    GridRobot(const GridConfig& grid, std::shared_ptr<Planner> planner = nullptr);

    /**
     * @brief Sense the environment.
     * 
     * This method retrieves the latest sensor readings from all sensors attached to the robot,
     * updates the `sensorDataCache_` with the data, and processes the information for further use.
     * 
     * @param state The current simulation state.
     */
    void sense(const SimulationState& state) override;

    /**
     * @brief Plan the next move using the assigned Planner.
     * 
     * - Syncs the robot's cache with the current SimulationState.
     * - Requests a new path from the Planner if the current path is empty or finished.
     * - Updates nextPos_ based on the planned path.
     * 
     * @param state The current simulation state.
     */
    void plan(SimulationState& state) override;

    /**
     * @brief Execute the planned move, updating the SimulationState.
     * 
     * - Moves along the planned path.
     * - Updates path_index_cache_ and currentPos_.
     * - Updates the SimulationState with the new position and path index.
     * 
     * @param state Current simulation state
     */
    void act(SimulationState& state) override;

    /**
     * @brief Syncs local cache with the current state of the robot in SimulationState.
     * 
     * @param state Current simulation state
     */
    void syncWithState(const SimulationState& state) override;

    /**
     * @brief Adds events related to the robot's actions or environment during the 
     *        current simulation tick.
     *
     * @param state Current simulation state.
     */
    void addEvents(SimulationState& state) const override;

    /**
     * @brief Check if the robot has reached its goal.
     * @param state The current simulation state.
     * 
     * @return True if the robot's current position equals its goal, false otherwise.
     */
    bool hasReachedGoal(const SimulationState& state) const;

    /**
     * @brief Set the planner strategy for the robot.
     * @param planner Shared pointer to the new planner.
     */
    void setPlanner(std::shared_ptr<Planner> planner);

    /**
     * @brief Get the current planner strategy of the robot.
     * 
     * @return A pointer to the current planner instance associated with the robot.
     */
    Planner* getPlanner() const override;

    /**
     * @brief Set the planned path for the robot.
     * 
     * @param path Vector of Positions representing the new path.
     * @param state The current simulation state.
     */
    void setPath(const std::vector<Position>& path, SimulationState& state);

    /**
     * @brief Get the current planned path of the robot.
     * 
     * @param state The current simulation state (unused, for interface compatibility)
     * 
     * @return Reference to the vector of Positions representing the path.
     */
    const std::vector<Position>& getPath([[maybe_unused]] const SimulationState& state) const;
    
    /**
     * @brief Get the current index in the planned path.
     * 
     * @param state The current simulation state (unused, for interface compatibility)
     * 
     * @return Index of the next step in the path.
     */
    size_t getPathIndex([[maybe_unused]] const SimulationState& state) const;
};