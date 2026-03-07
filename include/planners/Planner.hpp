#pragma once
#include "SimulationState.hpp"  // Forward Declaration
#include <vector>               // For std::vector

/**
 * @brief Abstract base class for all path-planning algorithms.
 * 
 * Each concrete planner (e.g., A*, Dijkstra, BFS) must inherit from this class
 * and implement the `computePath` method.
 */
class Planner
{
public:
    /**
     * @brief Compute a path for a robot from its current position to its goal.
     * 
     * This function calculates a sequence of positions that the robot should follow 
     * to reach its goal, considering obstacles and other robots in the simulation.
     * The planner does not update the robot's position. it only returns the planned path.
     * 
     * @param state The current simulation state, including robots and dynamic obstacles.
     * @param robot The state of the robot for which the path is being computed.
     * 
     * @return A sequence of positions representing the planned path.
     *         Returns an empty vector if no path is found.
     */
    virtual std::vector<Position> computePath(const SimulationState& state, const RobotState& robot) = 0;

    // Virtual destructor for proper cleanup of derived planner classes.
    virtual ~Planner() = default;
};