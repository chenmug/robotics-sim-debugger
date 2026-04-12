#include "planners/DijkstraPlanner.hpp"  // Forward Declaration


/**************** HEURISTIC ****************/

int DijkstraPlanner::heuristic(const Position& a, const Position& b) const
{
    return 0; // Dijkstra ignores heuristic
}


/*********** GET ALGORITHM NAME ************/

std::string DijkstraPlanner::getAlgorithmName() const
{
    return "DIJKSTRA";
}