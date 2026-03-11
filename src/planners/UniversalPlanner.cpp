#include "planners/UniversalPlanner.hpp"
#include <queue>          // For priority_queue
#include <cmath>          // For std::abs
#include <algorithm>      // For std::reverse


/*************** CONSTRUCTOR ***************/

UniversalPlanner::UniversalPlanner(PlannerType type)
    : plannerType_(type)
{}


/**************** HEURISTIC ****************/

int UniversalPlanner::heuristic(const Position& a, const Position& b) const
{
    if (plannerType_ == PlannerType::DIJKSTRA)
    {
        return 0;
    }

    return std::abs(a.x - b.x) + std::abs(a.y - b.y); // Manhattan distance
}


/**************** HASH POS *****************/

size_t UniversalPlanner::hashPos(const Position& pos, const GridConfig& grid) const
{
    return pos.y * grid.width + pos.x;
}


/************* RECONSTRUCT PATH ************/

std::vector<Position> UniversalPlanner::reconstructPath(const std::unordered_map<size_t, 
    Position>& cameFrom, const Position& start, const Position& goal, const GridConfig& grid) const
{
    std::vector<Position> path;
    Position node = goal;

    while (node != start)
    {
        path.push_back(node);
        node = cameFrom.at(hashPos(node, grid));
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}


/**************** COMPUTE PATH ****************/

std::vector<Position> UniversalPlanner::computePath(const SimulationState& state,
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