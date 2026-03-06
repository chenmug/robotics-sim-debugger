#include "core/SimulationEngine.hpp"
#include "robots/GridRobot.hpp"
#include <iostream>
#include <thread>
#include <chrono>


// TODO: move to GUI layer when switching to graphical rendering
void printGrid(const SimulationState& state, const GridConfig& grid)
{
    std::system("clear"); 

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
     std::cout << "Robotics Simulation Debugger - Console MVP\n";

    GridConfig grid{5, 5, {{2,2}, {3,1}}};
    SimulationEngine engine(grid);

    auto robot1 = std::make_unique<GridRobot>(1, Position{0,0}, Position{4,4}, 5, 5);
    engine.addRobot(std::move(robot1), {0,0}, {4,4});

    printGrid(engine.getCurrentState(), grid);

    while (engine.getCurrentState().robots[0].position != engine.getCurrentState().robots[0].goal)
    {
        engine.runTick();
        printGrid(engine.getCurrentState(), grid);
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
    }

    std::cout << "Robot reached goal!\n";
}