#include "controller/RobotModeBreakpoint.hpp"


// /*************** CONSTRUCTOR *****************/

RobotModeBreakpoint::RobotModeBreakpoint(size_t id, size_t robotId, RobotMode mode)
    : Breakpoint(id), robotId_(robotId), mode_(mode) {}



// /*************** SHOULD BREAK ****************/

bool RobotModeBreakpoint::shouldBreak([[maybe_unused]] const SimulationState& state, [[maybe_unused]] size_t tick) const
{
    for (const auto& robot : state.robots)
    {
        if (robot.id == robotId_ && robot.mode == mode_)
        {
            return true;
        }
    }

    return false;
} 