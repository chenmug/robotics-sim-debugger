#include "SimulationEngine.hpp" 


/**************** CONSTRUCTOR ****************/

SimulationEngine::SimulationEngine(const GridConfig& config)
    : grid(config) 
{
    current_state.tick = 0;
}


/***************** ADD ROBOT *****************/

void SimulationEngine::addRobot(std::unique_ptr<Robot> robot)
{
    robots.emplace_back(std::move(robot));
}


/****************** RUN TICK *****************/

void SimulationEngine::runTick()
{
    for (auto& robot : robots)
    {
        robot->sense(current_state);
    }

    for (auto& robot : robots)
    {
        robot->plan(current_state);
    }

    for (auto& robot : robots)
    {
        robot->act(current_state);
    }

    ++current_state.tick;
}


/************** GET CURRENT STATE *************/

const SimulationState& SimulationEngine::getCurrentState() const
{
    return current_state;
}