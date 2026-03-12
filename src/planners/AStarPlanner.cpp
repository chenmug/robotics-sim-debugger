#include "planners/AStarPlanner.hpp"
#include <cmath>  // For std::abs


/**************** HEURISTIC ****************/

int AStarPlanner::heuristic(const Position& a, const Position& b) const
{
    return std::abs(a.x - b.x) + std::abs(a.y - b.y); // Manhattan distance
}