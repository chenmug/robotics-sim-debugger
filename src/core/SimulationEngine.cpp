#include "core/SimulationEngine.hpp" 


/**************** CONSTRUCTOR ****************/

SimulationEngine::SimulationEngine(const GridConfig& config)
    : grid_(config) 
{
    current_state.tick = 0;
}


/***************** ADD ROBOT *****************/

void SimulationEngine::addRobot(std::unique_ptr<Robot> robot, Position start_pos)
{
    size_t id = current_state.robots.size();
    robot->setID(id);  // Assign unique ID to the robot before adding to engine
    RobotState state;

    state.id = id;
    state.position = start_pos;
    state.goal = start_pos;
    state.path_index = 0;
    state.planned_path.clear();

    current_state.robots.push_back(state);
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