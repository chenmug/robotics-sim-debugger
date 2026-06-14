#include "planners/Planner.hpp"       // Forward Declaration
#include "core/SimulationEngine.hpp"  // Forward Declaration
#include <algorithm>                  // For std::reverse


/**************** IS BLOCKED ****************/

bool Planner::isBlocked(const Position& pos, const SimulationState& state,
                   const GridConfig& grid, size_t self_id) const
{
    // Check static obstacles
    for (const auto& sObst : grid.static_obstacles)
    {
        if (sObst == pos)
        {
            return true;
        }
    }

    // Check robots
    for (const auto& other : state.robots)
    {
        // Ignore the robot currently planning
        if (other.id == self_id)
        {
            continue;
        }

        // Determine if the other robot is planning to move this tick
        bool otherWillMove = (other.nextPlannedPos != other.position);

        // Block only if the other robot is actually staying in place
        if (other.position == pos && !otherWillMove)
        {
            return true;
        }

        // Block the robot if a higher-priority robot plans to move into this cell
        if (other.id < self_id && otherWillMove && other.nextPlannedPos == pos)
        {
            return true;
        }
    }

    return false;
}


/**************** HASH POS *****************/

size_t Planner::hashPos(const Position& pos, const GridConfig& grid) const
{
    return pos.y * grid.width + pos.x;
}


/************* RECONSTRUCT PATH ************/

std::vector<Position> Planner::reconstructPath(const std::unordered_map<size_t, Position>& cameFrom,
        const Position& start, const Position& goal, const GridConfig& grid) const
{
    std::vector<Position> path;
    Position node = goal;

    while (node != start)
    {
        path.push_back(node);

        auto it = cameFrom.find(hashPos(node, grid));
        if (it == cameFrom.end())
        {
            return {};
        }

        node = it->second;
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}


/************ GET NODE EXPANDED ************/

size_t Planner::getNodesExpanded() const noexcept
{ 
    return lastNodesExpanded_; 
}


/************* GET LAST RUN TIMES ************/

double Planner::getLastRunTimeMs() const noexcept
{ 
    return lastRunTimeMs_; 
}