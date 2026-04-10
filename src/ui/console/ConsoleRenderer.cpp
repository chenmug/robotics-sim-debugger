#include "ui/console/ConsoleRenderer.hpp"
#include <iostream>


// /********** EVENT TYPE TO STRING *************/

std::string eventTypeToString(EventType type)
{
    switch(type)
    {
        case EventType::OBSTACLE_DETECTED: 
            return "ObstacleDetected";

        case EventType::REPLAN_TRIGGERED:  
            return "ReplanTriggered";

        case EventType::AVOID_COLLISION: 
            return "AvoidCollision";

        case EventType::GOAL_REACHED:
            return "GoalReached";

        default: return "UnknownEvent";
    }
}


// /********** ROBOT MODE TO STRING *************/

std::string robotModeToString(RobotMode mode)
{
    switch (mode)
    {
        case RobotMode::IDLE: 
            return "IDLE";

        case RobotMode::PLANNING: 
            return "PLANNING";

        case RobotMode::MOVING: 
            return "MOVING";

        case RobotMode::REPLANNING:
            return "REPLANNING";

        case RobotMode::GOAL: 
            return "GOAL";

        default: 
            return "UNKNOWN";
    }
}


// /*************** RENDER VIEW *****************/

void renderView(const DebugSnapshotView& view)
{
    renderHeader(view);
    renderGrid(view);
    renderRobots(view);
    renderEvents(view);
    renderFooter();
}


// /************** RENDER HEADER ****************/

void renderHeader(const DebugSnapshotView& view)
{
    std::cout << "==============================================================\n";
    std::cout << "Robotics Simulation Debugger\n";
    std::cout << "Tick: " << view.tick << "\n";
    std::cout << "Mode: " << (view.isRunning ? "RUNNING" : "PAUSED") << "\n";
    std::cout << "==============================================================\n\n";
}


// /*************** RENDER GRID *****************/

void renderGrid(const DebugSnapshotView& view)
{
    if (!view.state || !view.grid)
    {
        std::cout << "GRID: (no data)\n\n";
        return;
    }

    const auto& state = *view.state;
    const auto& grid = *view.grid;

    std::cout << "GRID:\n";

    for (int y = 0; y < grid.height; ++y)
    {
        for (int x = 0; x < grid.width; ++x)
        {
            Position pos{x, y};
            bool printed = false;

            // Robots
            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                if (state.robots[i].position == pos)
                {
                    std::cout << "R" << i + 1 << " ";
                    printed = true;
                    break;
                }
            }
            if (printed)
            { 
                continue;
            }

            // Static obstacles
            for (const auto& obst : grid.static_obstacles)
            {
                if (obst == pos)
                {
                    std::cout << "X ";
                    printed = true;
                    break;
                }
            }
            if (printed)
            { 
                continue;
            }

            // Dynamic obstacles
            for (const auto& obst : state.dynamic_obstacles)
            {
                if (obst == pos)
                {
                    std::cout << "X ";
                    printed = true;
                    break;
                }
            }
            if (printed)
            { 
                continue;
            }

            // Goals
            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                if (state.robots[i].goal == pos)
                {
                    std::cout << "G" << i + 1 << " ";
                    printed = true;
                    break;
                }
            }
            if (printed)
            { 
                continue;
            }

            std::cout << ". ";
        }

        std::cout << "\n";
    }

    std::cout << "\n";
}


// /************** RENDER ROBOTS ****************/

void renderRobots(const DebugSnapshotView& view)
{
    if (!view.state)
    {
        return;
    }

    const auto& state = *view.state;
    std::cout << "Robots Summary:\n";

    for (size_t i = 0; i < state.robots.size(); ++i)
    {
        const auto& r = state.robots[i];
        std::cout << "R" << i + 1
                  << " | Pos: (" << r.position.x << "," << r.position.y << ")"
                  << " | Goal: (" << r.goal.x << "," << r.goal.y << ")"
                  << " | Mode: " << robotModeToString(r.mode)
                  << "\n";
    }

    std::cout << "\n";
}


// /************** RENDER EVENTS ****************/

void renderEvents(const DebugSnapshotView& view)
{
    std::cout << "Events:\n";

    if (view.events.empty())
    {
        std::cout << " (none)\n\n";
        return;
    }

    for (const auto& e : view.events)
    {
        std::cout << " - " << e << "\n";
    }

    std::cout << "\n";
}


// /************** RENDER FOOTER ****************/

void renderFooter()
{
    std::cout << "===============================================================\n";
    std::cout << "[n] next | [b] back | [r] run | [p] pause | [j] jump | [q] quit\n";
    std::cout << "===============================================================\n";
}