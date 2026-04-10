#include "core/ConflictResolver.hpp"
#include "core/GridConfig.hpp"
#include <iostream>


/**************** RESOLVE *****************/

void ConflictResolver::resolve(SimulationState& state, const GridConfig& grid)
{
    size_t n = state.robots.size();

    resetBlockedNow(state);                 // Handle conflicts between all robot pairs
    detectAndHandleConflicts(state, grid);  // Update wasBlocked status
    updateBlockedStates(state);
}


/*********** RESET BLOCKED NOW ************/

void ConflictResolver::resetBlockedNow(SimulationState& state)
{
    for (auto& r : state.robots)
    {
        r.blockedNow = false;
    }
}


/****** DETECT AND HANDLE CONFLICTS *******/

void ConflictResolver::detectAndHandleConflicts(SimulationState& state, const GridConfig& grid)
{
    size_t n = state.robots.size();
    for (size_t i = 0; i < n; ++i)
    {
        auto& r1 = state.robots[i];
        if (r1.position == r1.goal)
        {
            continue;
        }
         
        for (size_t j = i + 1; j < n; ++j)
        {
            auto& r2 = state.robots[j];
            if (r2.position == r2.goal)
            {
                continue;
            } 

            const Position& next1 = r1.nextPlannedPos;
            const Position& next2 = r2.nextPlannedPos;

            bool isSwap = (next1 == r2.position && next2 == r1.position);
            bool sameCell = (next1 == next2);
            bool stuckConflict = (r1.wasBlocked && r2.wasBlocked && r1.position == r1.nextPlannedPos 
                                  && r2.position == r2.nextPlannedPos);

            if (isSwap || sameCell || stuckConflict)
            {
                handleConflict(r1, r2, state, grid, i, j);
            }
        }
    }
}


/************ HANDLE CONFLICT *************/

void ConflictResolver::handleConflict(RobotState& r1, RobotState& r2, SimulationState& state, const GridConfig& grid, size_t i, size_t j)
{
    bool firstTime = (!r1.wasBlocked && !r2.wasBlocked);

    if (firstTime)
    {
        r1.nextPlannedPos = r1.position;
        r2.nextPlannedPos = r2.position;
        r1.blockedNow = true;
        r2.blockedNow = true;
    }
    else
    {
        if (i < j)
        {
            moveAside(state, r2, grid);  // Move the low priority robot aside
        }
        else
        {
            moveAside(state, r1, grid);  // Move the low priority robot aside
        }
    }

    state.events.push_back({EventType::AVOID_COLLISION, i, state.tick});
    state.events.push_back({EventType::AVOID_COLLISION, j, state.tick});
}


/********* UPDATE BLOCKED STATES **********/

void ConflictResolver::updateBlockedStates(SimulationState& state)
{
    size_t n = state.robots.size();
    for (size_t i = 0; i < n; ++i)
    {
        auto& r = state.robots[i];
        r.wasBlocked = r.blockedNow;
    }
}


/*************** MOVE ASIDE ***************/

void ConflictResolver::moveAside(SimulationState& state, RobotState& robot, const GridConfig& grid)
{
    Position escape = findFreeNeighbor(state, robot, grid);

    robot.nextPlannedPos = escape;
    robot.blockedNow = true;
    robot.planned_path.clear(); // force replan
}


// /********** FIND FREE NEIGHBOOR ***********/

Position ConflictResolver::findFreeNeighbor(const SimulationState& state,
    const RobotState& robot, const GridConfig& grid) const
{
    for (const auto& d : grid.getDirections())
    {
        Position candidate{robot.position.x + d.x,robot.position.y + d.y};

        if (!grid.isWithinBounds(candidate))
        {
            continue;
        }

        bool occupied = false;

        for (const auto& other : state.robots)
        {
            if (other.position == candidate)
            {
                occupied = true;
                break;
            }
        }

        if (!occupied)
        {
            return candidate;
        }
    }

    return robot.position;
}