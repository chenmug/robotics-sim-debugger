#include "controller/TickBreakpoint.hpp"


// /*************** CONSTRUCTOR *****************/

TickBreakpoint::TickBreakpoint(size_t id, size_t tick)
    : Breakpoint(id), tick_(tick) {}


// /*************** SHOULD BREAK ****************/

bool TickBreakpoint::shouldBreak([[maybe_unused]] const SimulationState& state, [[maybe_unused]] size_t tick) const 
{

    return tick == tick_;
}