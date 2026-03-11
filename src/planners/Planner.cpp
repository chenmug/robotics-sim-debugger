#include "planners/Planner.hpp"       // Forward Declaration
#include "core/SimulationEngine.hpp"  // Forward Declaration


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

    // Check dynamic obstacles
    for (const auto& dObst : state.dynamic_obstacles)
    {
        if (dObst == pos)
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

        // Robot current position
        if (other.position == pos)
        {
            return true;
        }

        // Robot next planned step (collision avoidance)
        if (other.path_index < other.planned_path.size() &&
            other.planned_path[other.path_index] == pos)
        {
            return true;
        }
    }

    return false;
}


/************* IS WITHIN BOUNDS *************/

bool Planner::isWithinBounds(const Position& pos, const GridConfig& grid) const
{
    return pos.x >= 0 && pos.x < grid.width && pos.y >= 0 && pos.y < grid.height;
}