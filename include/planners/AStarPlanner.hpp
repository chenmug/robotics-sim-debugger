#pragma once
#include "planners/GraphSearchPlanner.hpp"  // Forward Declaration
#include <string>                           // For std::string


/**
 * @brief A* path planner for grid-based robots.
 * 
 * Implements the A* search algorithm by extending GraphSearchPlanner.
 * Uses a heuristic (Manhattan distance) to prioritize nodes closer to the goal.
 */
class AStarPlanner : public GraphSearchPlanner
{
public:
    
    /**
     * @brief Compute the heuristic cost between two positions for A*.
     * 
     * Uses Manhattan distance as the heuristic to estimate the cost from
     * position `a` to the goal `b`.
     *
     * @param a Current position
     * @param b Goal position
     * 
     * @return Estimated cost from `a` to `b` (Manhattan distance)
     */
    int heuristic(const Position& a, const Position& b) const override;

    // Return the name of the planner algorithm - A*.
    std::string getAlgorithmName() const override;
};