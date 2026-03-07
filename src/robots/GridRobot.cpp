#include "robots/GridRobot.hpp"


/**************** CONSTRUCTOR ****************/

GridRobot::GridRobot(int gridWidth, int gridHeight)
    : gridWidth_(gridWidth), gridHeight_(gridHeight){}


/******************* SENSE *******************/

void GridRobot::sense(const SimulationState& state)
{
    // MVP: no sensing yet
}


/******************* PLAN ********************/

void GridRobot::plan(SimulationState& state)
{
    Position nextPos = computeNextStep(state);
}


/******************** ACT ********************/

void GridRobot::act(SimulationState& state)
{
    Position nextPos = computeNextStep(state);

    if (isBounds(nextPos) && isFree(nextPos, state))
    {
        update(nextPos, state);
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
    RobotState& self = state.robots[id_];

    if (self.position != newPos)
    {
        self.position = newPos;
        self.path_index++;
    }
}


/************* COMPUTE NEXT STEP *************/

Position GridRobot::computeNextStep(const SimulationState& state) const
{
    const RobotState& self = state.robots[id_];
    Position nextPos = self.position;

    // Try moving in X direction first
    if (self.position.x != self.goal.x)
    {
        if (self.position.x < self.goal.x)
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
    nextPos = self.position; 

    if (self.position.y != self.goal.y)
    {
        if (self.position.y < self.goal.y)
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
    return self.position;
}


/************* HAS REACHED GOAL *************/

bool GridRobot::hasReachedGoal(const SimulationState& state) const
{
    const RobotState& self = state.robots[id_];

    return self.position == self.goal;
}