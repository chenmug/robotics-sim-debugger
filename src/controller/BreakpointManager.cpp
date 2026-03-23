#include "controller/Breakpoint.hpp"
#include "controller/BreakpointManager.hpp"
#include "controller/TickBreakpoint.hpp"
#include "controller/RobotModeBreakpoint.hpp"


// /*********** ADD TICK BREAKPOINT ***********/

size_t BreakpointManager::addTickBreakpoint(size_t tick)
{
    size_t id = nextBreakpointID_++;
    breakpoints_.push_back(std::make_unique<TickBreakpoint>(id, tick));

    return id;
}


// /********** ADD ROBOT BREAKPOINT ***********/

size_t BreakpointManager::addRobotBreakpoint(size_t robotId, RobotMode mode)
{
    size_t id = nextBreakpointID_++;
    breakpoints_.push_back(std::make_unique<RobotModeBreakpoint>(id, robotId, mode));

    return id;
}


// /************ REMOVE BREAKPOINT *************/

bool BreakpointManager::removeBreakpoint(size_t breakpointID)
{
    for (auto it = breakpoints_.begin(); it != breakpoints_.end(); ++it)
    {
        if ((*it)->getID() == breakpointID)
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


// /************** SHOULD BREAK ***************/

bool BreakpointManager::shouldBreak(const SimulationState& state, size_t tick) const
{
    for (const auto& bp : breakpoints_)
    {
        if (bp->shouldBreak(state, tick))
        {
            return true;
        }
    }

    return false;
}


// /************* GET BREAKPOINT **************/

const std::vector<std::unique_ptr<Breakpoint>>& BreakpointManager::getBreakpoints() const
{
    return breakpoints_;
}