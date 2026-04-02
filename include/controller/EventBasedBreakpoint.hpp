#pragma once
#include "Breakpoint.hpp"            // Forward Declaration
#include "core/SimulationState.hpp"  // Forward Declaration
#include <vector>                    // For std::vector


/**
 * @brief Breakpoint that triggers based on specific simulation events.
 *
 * This breakpoint pauses the simulation when one or more predefined events
 * occur during a simulation tick. It enables event-driven debugging, such as:
 * - A robot detecting an obstacle
 * - A robot triggering a path replanning
 * - Collisions between robots or with obstacles
 */
class EventBasedBreakpoint : public Breakpoint
{
private:
    std::vector<EventType> triggerEvents_;  // List of event types that trigger this breakpoint

public:
    /**
     * @brief Construct a new EventBasedBreakpoint.
     *
     * @param id Unique identifier for this breakpoint instance.
     * @param triggers A vector of EventType values that will trigger the breakpoint.
     */
    EventBasedBreakpoint(size_t id, const std::vector<EventType>& triggers);

    /**
     * @brief Check if the breakpoint condition is met.
     *
     * This override evaluates all events in the current SimulationState
     * and returns true if any event matches the predefined trigger list.
     *
     * @param state Current simulation state snapshot.
     * @param tick Current simulation tick.
     * @return True if any event triggers the breakpoint, false otherwise.
     */
    bool shouldBreak([[maybe_unused]] const SimulationState& state, [[maybe_unused]] size_t tick) const override;
};