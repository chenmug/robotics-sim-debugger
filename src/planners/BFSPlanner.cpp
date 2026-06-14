#include "planners/BFSPlanner.hpp"
#include <queue>   // For std::queue
#include <chrono>  // For timing


/**************** COMPUTE PATH ****************/

std::vector<Position> BFSPlanner::computePath(const SimulationState& state,
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

    if (start == goal)
    {
        lastRunTimeMs_ = 0.0;
        return {start};
    }

    const size_t gridSize = grid.width * grid.height;
    const size_t hashStart = hashPos(start, grid);

    std::queue<Position> openQueue;

    // Map to reconstruct path (key: hashed position, value: parent position)
    std::unordered_map<size_t, Position> cameFrom;

    // Track visited positions to avoid revisiting
    std::vector<bool> visited(gridSize, false);;

    openQueue.push(start);
    visited[hashStart] = true;

    while (!openQueue.empty())
    {
        Position current = openQueue.front();
        openQueue.pop();
        ++lastNodesExpanded_; // Count this node as expanded

        // If we reached the goal, reconstruct and return the path
        if (current == goal)
        {
            lastRunTimeMs_ = std::chrono::duration<double, std::milli>(Clock::now() - startTime).count();
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

            const size_t nHash = hashPos(neighbor, grid);
            if (!visited[nHash])
            {
                visited[nHash] = true;
                cameFrom[nHash] = current; // Record path
                openQueue.push(neighbor);
            }
        }
    }

    lastRunTimeMs_ = std::chrono::duration<double, std::milli>(Clock::now() - startTime).count();
    return {}; // No path found
}


/*********** GET ALGORITHM NAME ************/

std::string BFSPlanner::getAlgorithmName() const
{
    return "BFS";
}