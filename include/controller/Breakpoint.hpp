#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include <cstddef>                   // For size_t
#include <memory>                    // For std::unique_ptr


/**
 * @brief Abstract base class for all types of breakpoints.
 *
 * A breakpoint determines whether the simulation should pause given the
 * current SimulationState and tick.
 */
class Breakpoint
{
protected:

    size_t breakpointID_;  // Unique ID for this breakpoint

public:

    /**
     * @brief Construct a new Breakpoint with a unique ID.
     *
     * @param id Unique identifier for this breakpoint instance.
     */
    Breakpoint(size_t id) : breakpointID_(id) {}
    
    // Virtual destructor for proper cleanup of derived classes
    virtual ~Breakpoint() = default;

    /**
     * @brief Check if the breakpoint condition is met.
     *
     * This pure virtual function determines whether the simulation should
     * pause at the current state and tick. Derived classes implement the
     * specific condition logic.
     *
     * @param state Current simulation state (may be unused in some breakpoint types).
     * @param tick Current simulation tick (may be unused in some breakpoint types).
     * 
     * @return True if the simulation should pause, false otherwise.
     *
     * @note Both parameters are marked [[maybe_unused]] to allow derived
     *       classes to ignore them if not relevant.
     */
    virtual bool shouldBreak([[maybe_unused]] const SimulationState& state, [[maybe_unused]] size_t tick) const = 0;

    /**
     * @brief Get the unique ID of this breakpoint.
     */
    size_t getID() const { return breakpointID_; }
};
