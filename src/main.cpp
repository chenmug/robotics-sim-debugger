#include "controller/EngineController.hpp"
#include "core/SimulationEngine.hpp"
#include "robots/GridRobot.hpp"
#include "planners/AStarPlanner.hpp"
#include "planners/DijkstraPlanner.hpp"
#include "planners/BFSPlanner.hpp"
#include "controller/BreakpointManager.hpp"
#include "controller/EventBasedBreakpoint.hpp"
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
    auto planner4 = std::make_shared<AStarPlanner>();

    // ---------------------------
    // Robots
    // ---------------------------
    auto robot1 = std::make_unique<GridRobot>(grid, planner1);
    auto robot2 = std::make_unique<GridRobot>(grid, planner2);
    auto robot3 = std::make_unique<GridRobot>(grid, planner3);
    auto robot4 = std::make_unique<GridRobot>(grid, planner4);

    engine.addRobot(std::move(robot1), {0,0}, {9,9});
    engine.addRobot(std::move(robot2), {2,0}, {1,0});
    engine.addRobot(std::move(robot3), {9,0}, {0,9});
    engine.addRobot(std::move(robot4), {1,0}, {2,0});

    // Save initial state
    engine.getSnapshotManager().save(engine.getCurrentState());

    // ---------------------------
    // Engine controller
    // ---------------------------
    EngineController controller(engine, engine.getSnapshotManager());

    // ---------------------------
    // ADD BREAKPOINTS
    // ---------------------------
    // Pause at tick 17
    controller.getBreakpointManager().addTickBreakpoint(17);

    // Pause when certain events occur (event-based breakpoint)
    std::vector<EventType> triggerEvents = { EventType::AVOID_COLLISION};
    controller.getBreakpointManager().addEventBreakpoint(triggerEvents);

    controller.updateGUI();

    // ---------------------------
    // Main loop
    // ---------------------------
    while (true)
    {
        size_t tick = 0;

        if (controller.isFinished())
        {
            controller.quit();
            std::cout << "All robots reached their goals.\nSimulation ended.\n";
            break;
        }

        std::cout << "Commands: [n]ext, [b]ack, [r]un, [p]ause, [j]ump, [s]elect, [q]uit: ";
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

        case 'j':
        {
            std::cout << "Enter tick to jump to: ";
            std::cin >> tick;
            controller.jumpToTick(tick);
            
            if (controller.getCurrentTick() == tick)
            {
                std::cout << "Successfully jumped to tick " << tick << ".\n";
            }
            else
            {
                std::cout << "Jump interrupted at tick "  << controller.getCurrentTick() 
                          << " (breakpoint hit)\n";
            }
            
            break;
        }

        case 's':
        {
            int id;
            std::cout << "Select robot ID: ";
            std::cin >> id;

            controller.setSelectedRobot(id - 1);

            if (controller.getSelectedRobot() == -1)
            {
                std::cout << "Invalid robot selected\n";
            }
            else
            {
                std::cout << "Selected R" << id << "\n";
            }

            break;
        }
                
        default:
            std::cout << "Invalid command!\n" << std::endl;
            break;
        }
    }

    return 0;
}