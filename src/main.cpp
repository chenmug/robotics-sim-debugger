#include "controller/EngineController.hpp"
#include "core/SimulationEngine.hpp"
#include "robots/GridRobot.hpp"
#include "planners/AStarPlanner.hpp"
#include "planners/DijkstraPlanner.hpp"
#include "planners/BFSPlanner.hpp"
#include "controller/BreakpointManager.hpp"
#include <iostream>


// ======================================
// Main
// ======================================
int main()
{
    // ---------------------------
    // Grid configuration
    // ---------------------------
    GridConfig grid{10, 10, {{2,2}, {5,5}}};
    SimulationEngine engine(grid);

    // ---------------------------
    // Planners
    // ---------------------------
    auto planner1 = std::make_shared<AStarPlanner>();
    auto planner2 = std::make_shared<DijkstraPlanner>();
    auto planner3 = std::make_shared<BFSPlanner>();

    // ---------------------------
    // Robots
    // ---------------------------
    auto robot1 = std::make_unique<GridRobot>(grid, planner1);
    auto robot2 = std::make_unique<GridRobot>(grid, planner2);
    auto robot3 = std::make_unique<GridRobot>(grid, planner3);

    engine.addRobot(std::move(robot1), {0,0}, {9,9});
    engine.addRobot(std::move(robot2), {0,4}, {4,0});
    engine.addRobot(std::move(robot3), {9,0}, {0,9});

    // Save initial state
    engine.getSnapshotManager().save(engine.getCurrentState());

    // ---------------------------
    // Engine controller
    // ---------------------------
    EngineController controller(engine, engine.getSnapshotManager());


    // ---------------------------
    // ADD BREAKPOINTS
    // ---------------------------
    // Pause at tick 3
    controller.getBreakpointManager().addTickBreakpoint(3);

    // Pause when robot 1 enters IDLE mode
    controller.getBreakpointManager().addRobotBreakpoint(0, RobotMode::IDLE);

    controller.updateGUI();

    // ---------------------------
    // Main loop
    // ---------------------------
    while (true)
    {
        if (controller.isFinished())
        {
            std::cout << "All robots reached their goals.\nSimulation ended.\n";
            break;
        }

        std::cout << "Commands: [n]ext, [b]ack, [r]un, [p]ause, [q]uit: ";
        char cmd;
        std::cin >> cmd;

        switch (cmd)
        {
        case 'q':
            controller.quit();
            std::cout << "Quiting....\nSimulation ended.\n";
            return 0;

        case 'r':
            controller.run();
            break;
        
        case 'n':
            controller.stepForward();
            break;
        
        case 'b':
            controller.stepBack();
            break;
        
        case 'p':
            controller.pause();
            break;
        
        default:
            std::cout << "Invalid command!\n" << std::endl;
            break;
        }
    }

    return 0;
}