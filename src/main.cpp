#include "core/SimulationEngine.hpp"
#include "robots/GridRobot.hpp"
#include <iostream>
#include <thread>
#include <chrono>


// TODO: move to GUI layer when switching to graphical rendering
void printGrid(const SimulationState& state, const GridConfig& grid)
{
    std::system("clear"); 
    std::cout << "Robotics Simulation Debugger - Console MVP\n";

    std::cout << "Tick: " << state.tick << "\n\n";
    
    int x = 0;
    int y = 0;
    bool printed = true;
    
    for (y = 0; y < grid.height; ++y)
    {
        for (x = 0; x < grid.width; ++x)
        {
            Position pos{x, y};
            printed = false;

            for (const auto& robot : state.robots)
            {
                if (robot.position == pos)
                {
                    std::cout << "R ";
                    printed = true;
                    break;
                }
            }

            if (printed)
            { 
                continue;
            }

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

            for (const auto& dyn : state.dynamic_obstacles)
            {
                if (dyn == pos)
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

            for (const auto& robot : state.robots)
            {
                if (robot.goal == pos)
                {
                    std::cout << "G ";
                    printed = true;
                    break;
                }
            }

            if (!printed)
            { 
                std::cout << ". ";
            }
        }

        std::cout << "\n";
    }

    std::cout << "\n";
}



int main()
{
    GridConfig grid{10, 10, {{2,2}, {5,5}}};
    SimulationEngine engine(grid);

    auto robot1 = std::make_unique<GridRobot>(grid.width, grid.height);
    auto robot2 = std::make_unique<GridRobot>(grid.width, grid.height);

    engine.addRobot(std::move(robot1), {0,0}, {9,9});
    engine.addRobot(std::move(robot2), {0,4}, {4,0});

    printGrid(engine.getCurrentState(), grid);

    while (!engine.allRobotsReached())
    {
        engine.runTick();
        printGrid(engine.getCurrentState(), grid);

        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
    }

    std::cout << "All robots reached their goals!\n";
}