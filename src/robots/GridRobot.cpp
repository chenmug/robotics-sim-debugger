#include "robots/GridRobot.hpp"
#include "planners/Planner.hpp"
#include "sensors/Sensor.hpp"
#include "sensors/ObstacleSensor.hpp"
#include <iostream>


/**************** CONSTRUCTOR ****************/

GridRobot::GridRobot(const GridConfig& grid, std::shared_ptr<Planner> planner)
    : grid_(grid), planner_(planner)
{
    auto obstacleSensor = std::make_shared<ObstacleSensor>();
    addSensor(obstacleSensor);
}


/**************** SYNC CACHE ****************/

void GridRobot::syncWithState(const SimulationState& state)
{
    const RobotState& self = state.robots[id_];

    currentPos_ = self.position;
    goal_ = self.goal;

    planned_path_cache_ = self.planned_path;
    path_index_cache_ = self.path_index;

    if (!planned_path_cache_.empty() && path_index_cache_ < planned_path_cache_.size())
    {
        nextPos_ = planned_path_cache_[path_index_cache_];
    }
    else
    {
        nextPos_ = currentPos_;
    }

    mode_ = self.mode;
}


/******************* SENSE *******************/

void GridRobot::sense(const SimulationState& state)
{
    syncWithState(state);
    sensorDataCache_.clear();  // Clear previous sensor readings

    for (auto& sensor : sensors_)
    {
        if (!sensor)
        {
            continue;
        }

        SensorData data = sensor->read(state, grid_, id_);
        sensorDataCache_.push_back(data);
    }
}


/******************* PLAN ********************/

void GridRobot::plan(SimulationState& state)
{
    // Sync robot cache with current simulation state
    syncWithState(state);
    RobotState& self = state.robots[id_];
    
    // Change mode BEFORE planning
    self.mode = RobotMode::PLANNING;
    mode_ = self.mode;

    // Compute new path if planner exists and path is empty or finished
    if (planner_ && (planned_path_cache_.empty() || path_index_cache_ >= planned_path_cache_.size()))
    {
        planned_path_cache_ = planner_->computePath(state, self, grid_);
        path_index_cache_ = planned_path_cache_.empty() ? 0 : 1;

        self.planned_path = planned_path_cache_;
        self.path_index = path_index_cache_;
    }

    // Set next position from the planned path
    if (!planned_path_cache_.empty() && path_index_cache_ < planned_path_cache_.size())
    {
        nextPos_ = planned_path_cache_[path_index_cache_]; 
    }
    else
    {
        // Fallback: stay in place if no path
        nextPos_ = currentPos_;
    }

    // Updating the SimulationState with the next planned step
    self.nextPlannedPos = nextPos_;
}


/******************** ACT ********************/

void GridRobot::act(SimulationState& state)
{
    RobotState& self = state.robots[id_];
    
    // Change mode BEFORE moving
    if (self.position != self.nextPlannedPos)
    {
        self.position = self.nextPlannedPos;
        currentPos_ = self.position;

        self.mode = RobotMode::MOVING;
    }
    else
    {
        self.mode = RobotMode::REPLANNING;
    }

    mode_ = self.mode;

    if (self.position == self.goal)
    {
        self.mode = RobotMode::GOAL;
        mode_ = self.mode;
        return;
    }

    if (path_index_cache_ < planned_path_cache_.size())
    {
        ++path_index_cache_;
        self.path_index = path_index_cache_;
    }
}


/**************** ADD EVENTS *****************/

void GridRobot::addEvents(SimulationState& state) const
{
    size_t i = getID();

    // Check if goal reached
    if (currentPos_ == goal_)
    {
        state.events.push_back({EventType::GOAL_REACHED, i, state.tick});
        return;
    }

    // Check obstacles
    for (const auto& data : sensorDataCache_) 
    {
        if (!data.positions.empty()) 
        {
            state.events.push_back({EventType::OBSTACLE_DETECTED, i, state.tick});
            break;
        }
    }

    // Check replan
    if (state.robots[i].planned_path != planned_path_cache_) 
    {
        state.events.push_back({EventType::REPLAN_TRIGGERED, i, state.tick});
    }
}


/************* HAS REACHED GOAL *************/

bool GridRobot::hasReachedGoal(const SimulationState& state) const
{
    const RobotState& self = state.robots[id_];

    return self.position == self.goal;
}


/*************** SET PLANNER ***************/

void GridRobot::setPlanner(std::shared_ptr<Planner> planner)
{
    planner_ = planner;
}


/*************** GET PLANNER ***************/

std::shared_ptr<Planner> GridRobot::getPlanner() const
{
    return planner_;
}


/**************** SET PATH *****************/

void GridRobot::setPath(const std::vector<Position>& path, SimulationState& state)
{
    planned_path_cache_ = path;
    path_index_cache_ = 0;
    
    RobotState& self = state.robots[id_];
    self.planned_path = path;
    self.path_index = 0;

    // Update nextPos_ if path not empty
    nextPos_ = !planned_path_cache_.empty() ? planned_path_cache_[0] : currentPos_;
}


/**************** GET PATH *****************/

const std::vector<Position>& GridRobot::getPath([[maybe_unused]] const SimulationState& state) const
{
    return planned_path_cache_;
}


/************** GET PATH INDEX *************/

size_t GridRobot::getPathIndex([[maybe_unused]] const SimulationState& state) const
{
    return path_index_cache_;
}