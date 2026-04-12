#pragma once
#include "planners/Planner.hpp"       // Forward Declaration
#include "core/SimulationEngine.hpp"  // Forward Declaration
#include <string>                     // For std::string


/**
 * @brief Breadth-First Search planner for grid-based robots.
 *
 * Computes the shortest path in terms of number of steps (uniform cost) using BFS.
 */
class BFSPlanner : public Planner
{
public:

    /**
     * @brief Compute a path from the robot's current position to its goal.
     * 
     * @param state Current simulation state (robots and dynamic obstacles).
     * @param robot The robot state for which the path is computed.
     * @param grid Grid configuration.
     * 
     * @return Vector of Positions from start to goal, empty if no path found.
     */
    std::vector<Position> computePath(const SimulationState& state,
                        const RobotState& robot, const GridConfig& grid) override;

    // Return the name of the planner algorithm - BFS.
    std::string getAlgorithmName() const override;
};