#include "planners/GraphSearchPlanner.hpp"
#include <queue>          // For priority_queue


/**************** COMPUTE PATH ****************/

std::vector<Position> GraphSearchPlanner::computePath(const SimulationState& state,
    const RobotState& robot, const GridConfig& grid)
{
    const Position start = robot.position;
    const Position goal  = robot.goal;

    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> openSet;
    std::unordered_map<size_t, int> gScore;
    std::unordered_map<size_t, Position> cameFrom;

    int fScore = 0;
    const size_t startHash = hashPos(start, grid);
    gScore[startHash] = 0;
    openSet.push({heuristic(start, goal), start});

    while (!openSet.empty())
    {
        PQElement current = openSet.top();
        openSet.pop();

        if (current.pos == goal)
        {
            return reconstructPath(cameFrom, start, goal, grid);
        }

        for (const auto& dir : directions_)
        {
            Position neighbor{current.pos.x + dir.x, current.pos.y + dir.y};

            if (!isWithinBounds(neighbor, grid))
            {
                continue;
            }

            if (isBlocked(neighbor, state, grid, robot.id))
            {
                continue;
            }

            const int newCost = gScore[hashPos(current.pos, grid)] + 1;
            const size_t neighborHash = hashPos(neighbor, grid);

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
    return {};
}