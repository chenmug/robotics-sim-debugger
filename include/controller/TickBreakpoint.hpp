#include "controller/Breakpoint.hpp"  // Forward Declaration


/**
 * @brief Breakpoint that triggers when a specific simulation tick is reached.
 *
 * This breakpoint pauses the simulation when the current tick matches
 * the tick specified at construction.
 */
class TickBreakpoint : public Breakpoint
{
private:

    size_t tick_;  // The simulation tick at which this breakpoint triggers

public:

    /**
     * @brief Construct a new TickBreakpoint.
     *
     * @param id Unique identifier for this breakpoint instance.
     * @param tick The simulation tick at which the breakpoint should trigger.
     */
    TickBreakpoint(size_t id, size_t tick);

    /**
     * @brief Check if the breakpoint condition is met.
     *
     * @param state Current simulation state (not used in this breakpoint type).
     * @param tick Current simulation tick.
     * 
     * @return True if the current tick matches the breakpoint tick, false otherwise.
     *
     * @note The `state` parameter is marked [[maybe_unused]] since this
     *       breakpoint only depends on the tick.
     */
    bool shouldBreak([[maybe_unused]] const SimulationState& state, [[maybe_unused]] size_t tick) const override;
};