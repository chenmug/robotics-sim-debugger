#pragma once
#include "core/SimulationState.hpp"   // Forward Declaration
#include "core/SimulationEngine.hpp"  // For GridConfig
#include "planners/Planner.hpp"       // Forward Declaration
#include <unordered_map>              // For std::unordered_map
#include <vector>                     // For std::vecctor
#include <cstddef>                    // For size_t


/**
 * @brief Graph-based path planner for grid-based robots.
 *
 * GraphSearchPlanner implements generic graph-based search functionality for path planning,
 * designed to support multiple algorithms such as Dijkstra and A*.
 * Concrete planners (e.g., A*, Dijkstra) inherit from this class and implement the `heuristic()` function.
 *
 * Responsibilities:
 * - Compute a valid path from a robot's current position to its goal.
 * - Avoid static and dynamic obstacles as well as other robots.
 * - Handle grid boundaries.
 * - Supports only 4-directional movement (up, right, down, left).
 * - Return the planned path without modifying the SimulationState.
 */
class GraphSearchPlanner : public Planner
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
    std::vector<Position> computePath( const SimulationState& state,
                        const RobotState& robot, const GridConfig& grid) override;

protected:

    /**
     * @brief Node element for priority queue in the open set.
     * 
     * fScore = g + h (total estimated cost).
     * pos = grid position of this node.
     */
    struct PQElement 
    {
        int fScore;    // Total estimated cost (g + h)
        Position pos;  // Grid position of this node

        // Comparison operator for priority queue (min-heap)
        bool operator>(const PQElement& other) const 
        {
            return fScore > other.fScore;
        }
    };

    /**
     * @brief Heuristic function to override in derived classes.
     * 
     * Manhattan distance between two positions.
     * Returns 0 if using Dijkstra.
     * 
     * @param a Current position.
     * @param b Goal position.
     * 
     * @return Heuristic cost.
     */
    virtual int heuristic(const Position& a, const Position& b) const = 0;
};