#include "robots/GridRobot.hpp"


/**************** CONSTRUCTOR ****************/

GridRobot::GridRobot(size_t id, Position startPos, Position goalPos, int gridWidth, int gridHeight)
    : currentPos_(startPos), goal_(goalPos), gridWidth_(gridWidth), gridHeight_(gridHeight)
{
    setID(id);
}


/******************* SENSE *******************/

void GridRobot::sense(const SimulationState& state)
{
    // MVP: no sensing yet
}


/******************* PLAN ********************/

void GridRobot::plan(SimulationState& state)
{
    nextPos_ = computeNextStep(state);
}


/******************** ACT ********************/

void GridRobot::act(SimulationState& state)
{
    if (isBounds(nextPos_) && isFree(nextPos_, state))
    {
        update(nextPos_, state);
    }
}


/****************** IS FREE ******************/

bool GridRobot::isFree(Position pos, const SimulationState& state) const
{
    // Check if any robot occupies the position
    for (const auto& robot : state.robots)
    {
        if (robot.id != id_ && robot.position == pos)
        {
            return false;
        }
    }

    // Check if any dynamic obstacle occupies the position
    for (const auto& obst : state.dynamic_obstacles)
    {
        if (obst == pos)
        {
            return false;
        }
    }

    return true;
}


/***************** IS BOUNDS *****************/

bool GridRobot::isBounds(Position pos) const
{
    return pos.x >= 0 && pos.x < gridWidth_ && pos.y >= 0 && pos.y < gridHeight_;
}


/****************** UPDATE ******************/

void GridRobot::update(Position newPos, SimulationState& state)
{
    state.robots[id_].position = newPos;

    if (currentPos_ != newPos)
    {
        state.robots[id_].path_index++;
    }

    currentPos_ = newPos;
}


/************* COMPUTE NEXT STEP *************/

Position GridRobot::computeNextStep(const SimulationState& state) const
{
    Position nextPos = currentPos_;

    // Try moving in X direction first
    if (currentPos_.x != goal_.x)
    {
        if (currentPos_.x < goal_.x)
        {
            nextPos.x += 1;
        }
        else
        {
            nextPos.x -= 1;
        }
        
        if (isBounds(nextPos) && isFree(nextPos, state))
        {
            return nextPos;
        }
    }

    // If blocked or already aligned, try Y direction
    nextPos = currentPos_; 

    if (currentPos_.y != goal_.y)
    {
        if (currentPos_.y < goal_.y)
        {
            nextPos.y += 1;
        }
        else
        {
            nextPos.y -= 1;
        }

        if (isBounds(nextPos) && isFree(nextPos, state))
        {
            return nextPos;
        }
    }

    // If neither move is possible, stay in place
    return currentPos_;
}


/************* HAS REACHED GOAL *************/

bool GridRobot::hasReachedGoal() const
{
    return goal_ == currentPos_;
}