#pragma once
#include "robots/Robot.hpp"      // Forward Declaration
#include "planners/Planner.hpp"  // Forward declaration
#include <memory>                // For std::shared_ptr

/**
 * @brief A simple robot that navigates on a 2D grid towards a goal.
 * 
 * Implements the Robot interface for the MVP simulation:
 * - Computes next step per tick using the assigned Planner.
 * - Planner handles pathfinding, collision avoidance, and grid boundaries.
 * - GridRobot caches its current position, goal, and next step for efficiency.
 */
class GridRobot : public Robot
{
private:
    // Grid Info 
    int gridWidth_;    // The grid width.
    int gridHeight_;   // The grid height.

    // Local cache for robot convenience
    Position currentPos_;          // Current position (cache from SimulationState).
    Position goal_;                // Goal position (cache from SimulationState).
    Position nextPos_;             // Next step to move to.

    // Planner 
    std::shared_ptr<Planner> planner_;  // Planner strategy used by this robot.

public:
    /**
     * @brief Construct a GridRobot with grid size and optional planner.
     * @param gridWidth The width of the grid.
     * @param gridHeight The height of the grid.
     * @param planner Optional shared pointer to a Planner.
     */
    GridRobot(int gridWidth, int gridHeight, std::shared_ptr<Planner> planner = nullptr);

    /**
     * @brief Sense the environment (MVP: no sensing yet).
     * @param state The current simulation state.
     */
    void sense(const SimulationState& state) override;

    /**
     * @brief Plan the next action for the robot.
     *
     * Computes the next step along the path using the assigned Planner.
     * Takes into account dynamic and static obstacles and other robots in the current state.
     *
     * @param state The current simulation state.
     */
    void plan(SimulationState& state) override;

    /**
     * @brief Execute the planned action on the simulation state.
     *
     * Moves the robot to `nextPos_` if possible. If `nextPos_` is blocked, the robot
     * attempts to select an alternative step from the planned path.
     *
     * @param state The current simulation state.
     */
    void act(SimulationState& state) override;

    /**
     * @brief Check if the robot has reached its goal.
     * @param state The current simulation state.
     * 
     * @return True if the current position equals the goal, false otherwise.
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
     * @return Shared pointer to the current planner.
     */
    std::shared_ptr<Planner> getPlanner() const;

    /**
     * @brief Set the planned path for the robot.
     * 
     * @param path Vector of Positions representing the new path.
     * @param state The current simulation state.
     */
    void setPath(const std::vector<Position>& path, SimulationState& state);

    /**
     * @brief Get the current planned path of the robot.
     * @param state The current simulation state.
     * 
     * @return Reference to the vector of Positions representing the path.
     */
    const std::vector<Position>& getPath(const SimulationState& state) const;
    
    /**
     * @brief Get the current index in the planned path.
     * @param state The current simulation state.
     * 
     * @return Index of the next step in the path.
     */
    size_t getPathIndex(const SimulationState& state) const;
};