#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include <vector>                    // For std::vector
#include <unordered_map>             // For std::unordered_map
#include <string>                    // For std::string

struct GridConfig;

/**
 * @brief Abstract base class for all path-planning algorithms.
 *
 * Each concrete planner (e.g., A*, Dijkstra, BFS) must inherit from this class
 * and implement the computePath method.
 * 
 * This class tracks performance metrics for debugging and analysis:
 * - Number of nodes expanded during the last path computation.
 * - Duration of the last path planning run (in milliseconds).
 */
class Planner
{
protected:

    size_t lastNodesExpanded_ = 0;  // Number of nodes expanded in the last computePath call
    double lastRunTimeMs_ = 0.0;    // Duration of the last computePath call (milliseconds)

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

    /**
     * @brief Return the name of the planner algorithm (e.g., "A*", "BFS", "Dijkstra").
     */
    virtual std::string getAlgorithmName() const = 0;

    /**
     * @brief Get the number of nodes expanded during the last path planning run.
     *
     * @return Number of nodes expanded.
     */
    size_t getNodesExpanded() const;

    /**
     * @brief Get the duration of the last path planning run in milliseconds.
     *
     * @return Duration in milliseconds.
     */
    double getLastRunTimeMs() const;
    
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
     * @brief Hash a position to a unique non-negative integer.
     * 
     * Used to index positions in vectors/maps efficiently.
     * 
     * @param pos Position to hash.
     * @param grid Grid configuration.
     * 
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