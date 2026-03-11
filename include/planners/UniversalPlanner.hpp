#pragma once
#include "core/SimulationState.hpp"   // Forward Declaration
#include "core/SimulationEngine.hpp"  // For GridConfig
#include "planners/Planner.hpp"       // Forward Declaration
#include <unordered_map>              // For std::unordered_map
#include <vector>                     // For std::vecctor
#include <cstddef>                    // For size_t

/**
 * @brief Enum to select the type of path planning algorithm.
 * 
 * - DIJKSTRA: Uniform-cost search, guaranteed shortest path on uniform-cost grids.
 * - ASTAR: Uses heuristic (Manhattan distance) to guide search.
 */
enum class PlannerType 
{
    DIJKSTRA,
    ASTAR
};

/**
 * @brief Universal path planner for grid-based robots.
 * 
 * Implements both Dijkstra and A* search algorithms on a 2D grid.
 * Computes a path for a robot from its current position to its goal, 
 * avoiding static and dynamic obstacles.
 * 
 * Responsibilities:
 * - Return a planned path (does not modify SimulationState).
 * - Handle grid boundaries and obstacle checking.
 * - Supports only 4-directional movement (up, right, down, left).
 */
class UniversalPlanner : public Planner
{
public:
    /**
     * @brief Construct a new Universal Planner
     * 
     * @param type Algorithm type (DIJKSTRA or ASTAR). Default is ASTAR.
     */
    UniversalPlanner(PlannerType type = PlannerType::ASTAR);

    /**
     * @brief Compute a path from the robot's current position to its goal.
     * 
     * Executes the selected algorithm (Dijkstra or A*) and returns the planned path.
     * 
     * @param state Current simulation state (robots and dynamic obstacles).
     * @param robot The robot state for which the path is computed.
     * @param grid Grid configuration.
     * @return Vector of Positions from start to goal, empty if no path found.
     */
    std::vector<Position> computePath( const SimulationState& state,
                        const RobotState& robot, const GridConfig& grid) override;

private:
    PlannerType plannerType_;  // Selected planner type (DIJKSTRA or ASTAR)

    /**
     * @brief Node element for priority queue in the open set.
     * 
     * fScore = g + h (total estimated cost).
     * pos = grid position of this node.
     */
    struct PQElement 
    {
        int fScore;
        Position pos;

        bool operator>(const PQElement& other) const 
        {
            return fScore > other.fScore;
        }
    };

    // 4-directional move offsets: up, right, down, left
    const std::vector<Position> directions_ = {{0,1}, {1,0}, {0,-1}, {-1,0}};

    /**
     * @brief Heuristic function for A*.
     * 
     * Manhattan distance between two positions.
     * Returns 0 if using Dijkstra.
     * 
     * @param a Current position.
     * @param b Goal position.
     * @return Heuristic cost.
     */
    int heuristic(const Position& a, const Position& b) const;

    /**
     * @brief Hash a position to a unique non-negative integer.
     * 
     * Used to index positions in vectors/maps efficiently.
     * 
     * @param pos Position to hash.
     * @param grid Grid configuration.
     * @return Unique hash value.
     */
    size_t hashPos(const Position& pos, const GridConfig& grid) const;

    /**
     * @brief Reconstruct path from start to goal using cameFrom map
     * 
     * This function is used after the search algorithm (A* or Dijkstra) has completed. 
     * 
     * @param cameFrom A map where the key is the hashed position (size_t) and the value is the parent Position.
     *                 Each entry indicates from which node we arrived at the key node.
     * @param start The starting position of the robot.
     * @param goal The goal position of the robot.
     * @param grid The grid configuration.
     * 
     * @return A vector of positions representing the path from start to goal, including both start and goal. 
     *         The path is ordered sequentially. returns an empty vector if the goal is unreachable
     *         or missing in cameFrom.
     */
    std::vector<Position> reconstructPath(const std::unordered_map<size_t, Position>& cameFrom,
        const Position& start, const Position& goal, const GridConfig& grid) const;
};