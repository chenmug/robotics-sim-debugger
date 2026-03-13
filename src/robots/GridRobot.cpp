#include "robots/GridRobot.hpp"
#include "planners/Planner.hpp"

#define UNUSED(x) (void)x


/**************** CONSTRUCTOR ****************/

GridRobot::GridRobot(const GridConfig& grid, std::shared_ptr<Planner> planner)
    : grid_(grid), planner_(planner)
{}


/**************** SYNC CACHE ****************/

void GridRobot::syncWithState(const SimulationState& state)
{
    const RobotState& self = state.robots[id_];
    currentPos_ = self.position;
    goal_ = self.goal;
}


/******************* SENSE *******************/

void GridRobot::sense(const SimulationState& state)
{
    // MVP: no sensing logic yet
    syncWithState(state);
}


/******************* PLAN ********************/

void GridRobot::plan(const SimulationState& state)
{
    // Sync robot cache with current simulation state
    syncWithState(state);
    const RobotState& self = state.robots[id_];

    // Compute new path if planner exists and path is empty or finished
    if (planner_ && (planned_path_cache_.empty() || path_index_cache_ > planned_path_cache_.size()))
    {
        planned_path_cache_ = planner_->computePath(state, self, grid_);
        path_index_cache_ = 0;
    }

    // Update the path_index_cache to avoid getting stuck at the starting point
    if (path_index_cache_ == 0 && !planned_path_cache_.empty())
    {
        ++path_index_cache_; 
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
}


/******************** ACT ********************/

void GridRobot::act(SimulationState& state)
{
    RobotState& self = state.robots[id_];

    if (path_index_cache_ < planned_path_cache_.size())
    {
        currentPos_ = planned_path_cache_[path_index_cache_];
        self.position = currentPos_;
        ++path_index_cache_;
        self.path_index = path_index_cache_;
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

const std::vector<Position>& GridRobot::getPath(const SimulationState& state) const
{
    UNUSED(state);

    return planned_path_cache_;
}


/************** GET PATH INDEX *************/

size_t GridRobot::getPathIndex(const SimulationState& state) const
{
    UNUSED(state);
    
    return path_index_cache_;
}