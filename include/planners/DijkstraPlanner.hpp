#pragma once
#include "planners/GraphSearchPlanner.hpp"  // Forward Declaration
#include <string>                           // For std::string


/**
 * @brief Dijkstra path planner for grid-based robots.
 * 
 * Computes the shortest path from a robot's current position to its goal 
 * using Dijkstra's algorithm (uniform-cost search).
 * Dijkstra's algorithm does not use a heuristic, so this function
 * always returns 0 regardless of the positions.
 */
class DijkstraPlanner : public GraphSearchPlanner
{
public:

    // Heuristic function for Dijkstra's algorithm - always returns 0.
    int heuristic(const Position& a, const Position& b) const override;

    // Return the name of the planner algorithm - Dijkstra.
    std::string getAlgorithmName() const override;
};