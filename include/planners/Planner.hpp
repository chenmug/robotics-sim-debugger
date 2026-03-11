#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include <vector>                    // For std::vector

struct GridConfig;

/**
 * @brief Abstract base class for all path-planning algorithms.
 *
 * Each concrete planner (e.g., A*, Dijkstra, BFS) must inherit from this class
 * and implement the computePath method.
 */
class Planner
{
public:

    /**
     * @brief Compute a path for a robot from its current position to its goal.
     *
     * The planner does not modify the simulation state. It only computes and
     * returns a path that the robot can follow.
     *
     * The returned path contains the sequence of **next positions** the robot
     * should move through to reach the goal (excluding the current position).
     *
     * @param state Current simulation state
     * @param robot Robot requesting the path
     * @param grid Grid configuration
     *
     * @return Vector of positions representing the path
     */
    virtual std::vector<Position> computePath(const SimulationState& state,
                                const RobotState& robot,const GridConfig& grid) = 0;
    
    // Virtual destructor for proper cleanup of derived classes
    virtual ~Planner() = default;

protected:

    /**
     * @brief Check if a position is blocked.
     *
     * A position is considered blocked if it contains:
     * - a static obstacle
     * - a dynamic obstacle
     * - another robot
     * 
     * @param pos The position to check.
     * @param state Current simulation state.
     * @param grid Grid configuration.
     * @param self_id The unique identifier of the robot.
     *
     * @return True if the position is block, false otherwise.
     */
    bool isBlocked(const Position& pos, const SimulationState& state,
                   const GridConfig& grid, size_t self_id) const;

    /**
     * @brief Check if a position is within the grid boundaries.
     * @param pos The position to check.
     * @param grid Grid configuration.
     *
     * @return True if the position is in the grid boundaries, false otherwise.
     */
    bool isWithinBounds(const Position& pos, const GridConfig& grid) const;
};