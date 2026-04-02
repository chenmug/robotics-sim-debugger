#include "controller/EventBasedBreakpoint.hpp"


// /*************** CONSTRUCTOR *****************/

EventBasedBreakpoint::EventBasedBreakpoint(size_t id, const std::vector<EventType>& triggers)
    : Breakpoint(id), triggerEvents_(triggers) {}


// /*************** SHOULD BREAK ****************/

bool EventBasedBreakpoint::shouldBreak([[maybe_unused]] const SimulationState& state, [[maybe_unused]] size_t tick) const
{
    for (const auto& event : state.events)
    {
        for (const auto& trigger : triggerEvents_)
        {
            if (event.type == trigger)
            {
                return true;
            }
        }
    }

    return false;
}