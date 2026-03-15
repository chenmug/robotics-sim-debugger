#include "sensors/ObstacleSensor.hpp"
#include <cmath>  // For std::sqrt


/**************** CONSTRUCTOR ****************/

ObstacleSensor::ObstacleSensor(int range)
    : range_(range) {}


/******************* READ *******************/

SensorData ObstacleSensor::read(const SimulationState& state, const GridConfig& grid, size_t robotID) 
{
    SensorData data;
    const auto& robot = state.robots[robotID];
    Position pos = robot.position;
    size_t i = 0;

    if (robotID >= state.robots.size())
    {
        return data;
    } 

    // Static obstacles
    for (const auto& obst : grid.static_obstacles) 
    {
        addObstacleIfInRange(obst, pos, data);
    }

    // Dynamic obstacles
    for (const auto& dyn : state.dynamic_obstacles) 
    {
        addObstacleIfInRange(dyn, pos, data);
    }

    // Other robots
    for (i = 0; i < state.robots.size(); ++i) 
    {
        if (i == robotID)
        {
            continue;
        }

        const auto& other = state.robots[i];
        addObstacleIfInRange(other.position, pos, data);
    }

    return data;
}


/***************** GET NAME *****************/

std::string ObstacleSensor::getName() const
{
    return "ObstacleSensor";
}


/********* ADD OBSTACLE IF IN RANGE *********/

void ObstacleSensor::addObstacleIfInRange(const Position& obstaclePos, const Position& robotPos, SensorData& data)
{
    int dx = obstaclePos.x - robotPos.x;
    int dy = obstaclePos.y - robotPos.y;

    if (dx * dx + dy * dy <= range_ * range_) 
    {
        data.positions.push_back(obstaclePos);
    }
}