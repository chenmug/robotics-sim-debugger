#include "planners/GraphSearchPlanner.hpp"
#include <queue>   // For priority_queue
#include <chrono>  // For timing


/**************** COMPUTE PATH ****************/

std::vector<Position> GraphSearchPlanner::computePath(const SimulationState& state,
    const RobotState& robot, const GridConfig& grid)
{
    using Clock = std::chrono::high_resolution_clock;
    auto startTime = Clock::now();

    lastNodesExpanded_ = 0; // Reset counter
    const Position start = robot.position;
    const Position goal  = robot.goal;

    // Check if start and goal is within bounds
    if (!grid.isWithinBounds(start) || !grid.isWithinBounds(goal))
    {
        return {};
    }

    // Open set (priority queue) ordered by fScore (g + h)
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> openSet;
    std::unordered_map<size_t, int> gScore;
    std::unordered_map<size_t, Position> cameFrom;

    // Initial fScore for the start node
    int fScore = 0;
    const size_t startHash = hashPos(start, grid);
    gScore[startHash] = 0;

    // Add the start node to the open set with its fScore
    openSet.push({heuristic(start, goal), start});

    while (!openSet.empty())
    {
        PQElement current = openSet.top();
        openSet.pop();
        ++lastNodesExpanded_; // Count this node as expanded

        // If we reached the goal, reconstruct and return the path
        if (current.pos == goal)
        {
            lastRunTimeMs_ = std::chrono::duration<double, std::milli>(Clock::now() - startTime).count();
            return reconstructPath(cameFrom, start, goal, grid);
        }

        // Explore neighbors (4 directions)
        for (const auto& dir : grid.getDirections())
        {
            Position neighbor{current.pos.x + dir.x, current.pos.y + dir.y};
            
            // Skip neighbor if it's out of bounds or blocked
            if (!grid.isWithinBounds(neighbor) || isBlocked(neighbor, state, grid, robot.id))
            {
                continue;
            }

            const int newCost = gScore[hashPos(current.pos, grid)] + 1;
            const size_t neighborHash = hashPos(neighbor, grid);
            
            // If neighbor is not in gScore or we found a cheaper path
            if (!gScore.count(neighborHash) || newCost < gScore[neighborHash])
            {
                cameFrom[neighborHash] = current.pos;
                gScore[neighborHash] = newCost;
                
                fScore = newCost + heuristic(neighbor, goal);
                openSet.push({fScore, neighbor});
            }
        }
    }

    // No path found
    lastRunTimeMs_ = std::chrono::duration<double, std::milli>(Clock::now() - startTime).count();
    return {};
}