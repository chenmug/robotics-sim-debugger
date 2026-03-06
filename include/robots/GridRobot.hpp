#pragma once
#include "Robot.hpp"  // Forward Declaration

/**
 * @brief A simple robot that navigates on a 2D grid towards a goal.
 * 
 * Implements the Robot interface for the MVP simulation:
 * - Moves one step per tick in X-first, then Y direction.
 * - Checks grid boundaries and collisions with obstacles.
 */
class GridRobot : public Robot
{
private:
    Position currentPos_; // Current position of the robot.
    Position nextPos_;    // The next position the robot plans to move to.
    Position goal_;       // Target goal position.
    int gridWidth_;       // The grid width.
    int gridHeight_;      // The grid height.

public:
    /**
     * @brief Construct a GridRobot with a unique ID and initial position.
     * @param id Unique identifier for this robot.
     * @param startPos Starting position on the grid.
     * @param goalPos Goal position the robot should reach.
     */
    GridRobot(size_t id, Position startPos, Position goalPos);

    /**
     * @brief Sense the environment (MVP: no sensing yet).
     * @param state The current simulation state.
     */
    void sense(const SimulationState& state) override;

    /**
     * @brief Plan the next action (computes next step).
     * @param state The current simulation state.
     */
    void plan(SimulationState& state) override;

    /**
     * @brief Act on the simulation state (updates position if possible).
     * @param state The current simulation state.
     */
    void act(SimulationState& state) override;

    /**
     * @brief Check if a given position is free of robots and obstacles.
     * @param pos The position the robot wants to move to.
     * @param state The current simulation state.
     * 
     * @return True if the position is free, false otherwise.
     */
    bool isFree(Position pos, const SimulationState& state) const;

    /**
     * @brief Check if the position is within the grid boundaries.
     * @param pos The position to check.
     * 
     * @return True if the position is inside the grid, false otherwise.
     */
    bool isBounds(Position pos) const;

    /**
     * @brief Update the robot's position in the SimulationState.
     * @param newPos The new position to move the robot to.
     * @param state The current simulation state.
     */
    void update(Position newPos, SimulationState& state);

    /**
     * @brief Compute the next step of the robot based on X-first/Y-second strategy.
     * @param state The current simulation state.
     * 
     * @return The next position the robot will attempt to move to.
     */
    Position computeNextStep(const SimulationState& state) const;

    /**
     * @brief Check if the robot has reached its goal.
     * 
     * @return True if the current position equals the goal, false otherwise.
     */
    bool hasReachedGoal() const;
};