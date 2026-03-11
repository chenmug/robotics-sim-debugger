#include "core/SimulationEngine.hpp"
#include "robots/GridRobot.hpp"
#include "planners/UniversalPlanner.hpp"
#include <thread>
#include <chrono>
#include <iostream>


// TODO: move to GUI layer when switching to graphical rendering
void printGrid(const SimulationState& state, const GridConfig& grid)
{
    std::system("clear"); 
    std::cout << "Robotics Simulation Debugger - Console MVP\n";
    std::cout << "Tick: " << state.tick << "\n\n";

    for (int y = 0; y < grid.height; ++y)
    {
        for (int x = 0; x < grid.width; ++x)
        {
            Position pos{x, y};
            bool printed = false;

            // Robots
            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                const auto& robot = state.robots[i];
                if (robot.position == pos)
                {
                    std::cout << "R" << i+1 << " ";
                    printed = true;
                    break;
                }
            }
            if (printed) continue;

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
            if (printed) continue;

            // Dynamic obstacles
            for (const auto& dyn : state.dynamic_obstacles)
            {
                if (dyn == pos)
                {
                    std::cout << "X ";
                    printed = true;
                    break;
                }
            }
            if (printed) continue;

            // Goals
            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                const auto& robot = state.robots[i];
                if (robot.goal == pos)
                {
                    std::cout << "G" << i + 1 << " ";
                    printed = true;
                    break;
                }
            }
            if (printed) continue;

            std::cout << ". ";
        }
        std::cout << "\n";
    }

    std::cout << "\n";
}



int main()
{
    // Grid configuration
    GridConfig grid{10, 10, {{2,2}, {5,5}}};
    SimulationEngine engine(grid);

    // Planners
    auto planner1 = std::make_shared<UniversalPlanner>(PlannerType::ASTAR);
    auto planner2 = std::make_shared<UniversalPlanner>(PlannerType::DIJKSTRA);
    auto planner3 = std::make_shared<UniversalPlanner>(PlannerType::ASTAR);

    // Robots
    auto robot1 = std::make_unique<GridRobot>(grid, planner1);
    auto robot2 = std::make_unique<GridRobot>(grid, planner2);
    auto robot3 = std::make_unique<GridRobot>(grid, planner3);

    // Add robots to simulation
    engine.addRobot(std::move(robot1), {0,0}, {9,9});
    engine.addRobot(std::move(robot2), {0,4}, {4,0});
    engine.addRobot(std::move(robot3), {9,0}, {0,9});

    // Main simulation loop
    while (!engine.allRobotsReached())
    {
        // Run one tick
        engine.runTick();
        printGrid(engine.getCurrentState(), grid);

        // Pause for visualization
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "All robots reached their goals!\n";
}