#include "planners/BFSPlanner.hpp"
#include <queue>  // For std::queue


/**************** COMPUTE PATH ****************/

std::vector<Position> BFSPlanner::computePath(const SimulationState& state,
                                    const RobotState& robot, const GridConfig& grid) 
{
    std::queue<Position> openQueue;

    // Map to reconstruct path (key: hashed position, value: parent position)
    std::unordered_map<size_t, Position> cameFrom;

    // Track visited positions to avoid revisiting
    std::unordered_map<size_t, bool> visited;

    Position start = robot.position;
    Position goal  = robot.goal;

    size_t nHash = 0;

    // Check if start and goal is within bounds
    if (!grid.isWithinBounds(start) || !grid.isWithinBounds(goal))
    {
        return {};
    }

    openQueue.push(start);
    visited[hashPos(start, grid)] = true;

    while (!openQueue.empty())
    {
        Position current = openQueue.front();
        openQueue.pop();

        // If we reached the goal, reconstruct and return the path
        if (current == goal)
        {
            return reconstructPath(cameFrom, start, goal, grid);
        }

        // Explore neighbors (4 directions)
        for (const auto& dir : grid.getDirections())
        {
            Position neighbor{current.x + dir.x, current.y + dir.y};

            // Skip neighbor if it's out of bounds or blocked
            if (!grid.isWithinBounds(neighbor) || isBlocked(neighbor, state, grid, robot.id))
            {
                continue;
            }

            nHash = hashPos(neighbor, grid);
            if (!visited[nHash])
            {
                visited[nHash] = true;
                cameFrom[nHash] = current; // Record path
                openQueue.push(neighbor);
            }
        }
    }

    return {}; // No path found
}