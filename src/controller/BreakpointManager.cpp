#include "controller/BreakpointManager.hpp"


// /*********** ADD TICK BREAKPOINT ***********/

size_t BreakpointManager::addTickBreakpoint(size_t tick)
{
    Breakpoint tickBreakpoint;
    tickBreakpoint.breakpointID = nextBreakpointID_++;
    tickBreakpoint.tick = tick;
    tickBreakpoint.type = BreakpointType::TICK;
    
    breakpoints_.push_back(tickBreakpoint);

    return tickBreakpoint.breakpointID;
}


// /********** ADD ROBOT BREAKPOINT ***********/

size_t BreakpointManager::addRobotBreakpoint(size_t robotId, RobotMode mode)
{
    Breakpoint robotBreakpoint;
    robotBreakpoint.breakpointID = nextBreakpointID_++;
    robotBreakpoint.robotId = robotId;
    robotBreakpoint.mode = mode;
    robotBreakpoint.type = BreakpointType::ROBOT_MODE;
    
    breakpoints_.push_back(robotBreakpoint);

    return robotBreakpoint.breakpointID;
}


// /************ REMOVE BREAKPOINT *************/

bool BreakpointManager::removeBreakpoint(size_t breakpointID)
{
    for (auto it = breakpoints_.begin(); it != breakpoints_.end(); ++it)
    {
        if (it->breakpointID == breakpointID)
        {
            breakpoints_.erase(it);
            return true;
        }
    }

    return false;
}


// /************ CLEAR BREAKPOINT *************/

void BreakpointManager::clearAllBreakpoints()
{
    breakpoints_.clear();
}


// /*********** IS TICK BREAKPOINT ************/

bool BreakpointManager::isTickBreakpoint(size_t currentTick) const
{
    for (const auto& bp : breakpoints_)
    {
        if (bp.type == BreakpointType::TICK && bp.tick.has_value() 
            && bp.tick.value() == currentTick)
        {
            return true;
        }
    }

    return false;
}


// /********** IS ROBOT BREAKPOINT ************/

bool BreakpointManager::isRobotBreakpoint(size_t robotId, RobotMode mode) const
{
    for (const auto& bp : breakpoints_)
    {
        if (bp.type == BreakpointType::ROBOT_MODE &&  bp.robotId.has_value()
            && bp.robotId.value() == robotId && bp.mode.has_value() && 
            bp.mode.value() == mode)
        {
            return true;
        }
    }

    return false;
}


// /************* GET BREAKPOINT **************/

const std::vector<Breakpoint>& BreakpointManager::getBreakpoints() const
{
    return breakpoints_;
}