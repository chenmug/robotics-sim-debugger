#include "core/SimulationEngine.hpp"
#include "robots/GridRobot.hpp"
#include <iostream>

int main() 
{
    std::cout << "Robotics Simulation Debugger - Console MVP\n";
    
    GridConfig grid {5, 5, std::vector<Position>{{2,2}, {3,1}}}; 
    SimulationEngine engine(grid);

    auto gRobot = std::make_unique<GridRobot>(1, Position{0,0}, Position{4,4}, 5, 5);

    engine.addRobot(std::move(gRobot), Position{0,0});

    for (int i = 0; i < 10; ++i) 
    {
        engine.runTick();

        const auto& state = engine.getCurrentState();
        const auto& robotPos = state.robots[0].position;

        std::cout << "Tick " << state.tick 
                  << ": Robot at (" << robotPos.x << "," << robotPos.y << ")" << std::endl;

        if (state.robots[0].position == Position{4,4}) 
        {
            std::cout << "Robot reached goal!" << std::endl;
            break;
        }
    }

    return 0;
}