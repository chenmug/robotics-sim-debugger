#include "core/SimulationEngine.hpp"
#include "robots/GridRobot.hpp"
#include "planners/AStarPlanner.hpp"
#include "planners/DijkstraPlanner.hpp"
#include "planners/BFSPlanner.hpp"
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
                    std::cout << "R" << i + 1 << " ";
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
    auto planner1 = std::make_shared<AStarPlanner>();
    auto planner2 = std::make_shared<DijkstraPlanner>();
    auto planner3 = std::make_shared<BFSPlanner>();

    // Robots
    auto robot1 = std::make_unique<GridRobot>(grid, planner1);
    auto robot2 = std::make_unique<GridRobot>(grid, planner2);
    auto robot3 = std::make_unique<GridRobot>(grid, planner3);

    // Add robots to simulation
    engine.addRobot(std::move(robot1), {0,0}, {9,9});
    engine.addRobot(std::move(robot2), {0,4}, {4,0});
    engine.addRobot(std::move(robot3), {9,0}, {0,9});

    size_t currentTick = 0;
    engine.getSnapshotManager().save(engine.getCurrentState());

    while (!engine.allRobotsReached())
    {
        // Fetch the current snapshot
        const SimulationState* state = engine.getSnapshotManager().get(currentTick);
        if (!state) 
        {
            break;
        }

        printGrid(*state, grid);

        // User input for time travel
        std::cout << "Commands: [n]ext, [b]ack, [q]uit: ";
        char cmd;
        std::cin >> cmd;

        if (cmd == 'q') break;
        else if (cmd == 'n') 
        {
            if (currentTick + 1 >= engine.getSnapshotManager().getSize())
            {
                engine.runTick();  // Advance simulation only if at latest tick
            }
            if (currentTick + 1 < engine.getSnapshotManager().getSize())
            {
                ++currentTick;
            }
        }
        else if (cmd == 'b') 
        {
            if (engine.getSnapshotManager().canStepBack(currentTick))
            {
                --currentTick;
            }
        }
        else
        {
            std::cout << "Invalid command.\n";
        }

        // Slow down for visualization
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    printGrid(engine.getCurrentState(), grid);
    std::cout << "Simulation ended.\n";
}