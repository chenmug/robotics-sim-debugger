#include "sensors/GoalSensor.hpp"


/******************* READ *******************/

SensorData GoalSensor::read(const SimulationState& state, const GridConfig& grid, size_t robotID)
{
    SensorData data;

    if (robotID < state.robots.size())
    {
        data.positions.push_back(state.robots[robotID].goal);
    }

    return data;
}


/***************** GET NAME *****************/

std::string GoalSensor::getName() const
{
    return "GoalSensor";
}