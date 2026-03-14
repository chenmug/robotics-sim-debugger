#include "core/SimulationEngine.hpp" 
#include <stdexcept>  // For std::runtime_error


/**************** CONSTRUCTOR ****************/

SimulationEngine::SimulationEngine(const GridConfig& config)
    : grid_(config) 
{
    current_state.tick = 0;
}


/***************** ADD ROBOT *****************/

void SimulationEngine::addRobot(std::unique_ptr<Robot> robot, Position start_pos, Position goal_pos)
{
    const size_t id = current_state.robots.size();
    robot->setID(id);  // Assign unique ID to the robot before adding to engine

    if (!isWithinBounds(start_pos) || !isWithinBounds(goal_pos))
    {
        throw std::runtime_error("Robot position outside grid bounds");
    }
    
    // Initialize robot state
    RobotState state;
    state.id = id;
    state.position = start_pos;
    state.goal = goal_pos;
    state.path_index = 0;
    state.planned_path.clear();

    // Add to engine
    current_state.robots.push_back(state);
    robots.emplace_back(std::move(robot));
}


/****************** RUN TICK *****************/

void SimulationEngine::runTick()
{
    ++current_state.tick;
    
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

    snapshotManager_.save(current_state);
}


/************** GET CURRENT STATE *************/

const SimulationState& SimulationEngine::getCurrentState() const
{
    return current_state;
}


/************** GET GRID CONFIG *************/

const GridConfig& SimulationEngine::getGridConfig() const
{
    return grid_;
}


/************* IS VALID POSITION ************/

bool SimulationEngine::isWithinBounds(const Position& pos) const
{
    return pos.x >= 0 && pos.x < grid_.width && pos.y >= 0 && pos.y < grid_.height;
}


/************ ALL ROBOTS REACHED ************/

bool SimulationEngine::allRobotsReached() const
{
    for (const auto& robot : current_state.robots)
    {
        if (robot.position != robot.goal)
        {
            return false;
        }
    }

    return true;
}


/************* GET ROBOT COUNT *************/

size_t SimulationEngine::getRobotCount() const
{
    return robots.size();
}


/********** GET SNAPSHOT MANAGER ***********/

SnapshotManager& SimulationEngine::getSnapshotManager() 
{
    return snapshotManager_;
}