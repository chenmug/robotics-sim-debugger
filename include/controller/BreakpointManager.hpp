#pragma once
#include "core/SimulationState.hpp"   // Forward Declaration
#include "controller/Breakpoint.hpp"  // Forward Declaration
#include "robots/Robot.hpp"           // Forward Declaration
#include <vector>                     // For std::vector
#include <cstddef>                    // For size_t
#include <queue>                      // For std::queue


/**
 * @brief Manages breakpoints for the simulation debugger.
 *
 * Provides an interface to add, remove, query, and clear breakpoints.
 * Intended to be used by EngineController to automatically pause the simulation
 * when a breakpoint condition is met.
 */
class BreakpointManager
{
private:

    std::vector<std::unique_ptr<Breakpoint>> breakpoints_;  // Stored breakpoints
    size_t nextBreakpointID_ = 0;                           // Next unique ID for breakpoints
    std::queue<size_t> removedIDs_;                         // Restore the removed IDs

public:
    
    // Construct a new BreakpointManager.
    BreakpointManager() = default;

    // Destroy the BreakpointManager.
    ~BreakpointManager() = default;

    /**
     * @brief Add a tick-based breakpoint.
     *
     * @param tick Simulation tick at which the simulation should pause.
     * 
     * @return The unique ID of the newly created breakpoint.
     */
    size_t addTickBreakpoint(size_t tick);

    /**
     * @brief Add a robot state-based breakpoint.
     *
     * The simulation will pause when the specified robot enters the given mode.
     *
     * @param robotId ID of the robot to monitor.
     * @param mode RobotMode that triggers the breakpoint.
     * 
     * @return The unique ID of the newly created breakpoint.
     */
    size_t addRobotBreakpoint(size_t robotId, RobotMode mode);

    /**
     * @brief Add an event-based breakpoint.
     *
     * The simulation will pause when any of the specified events occur.
     *
     * @param triggers Vector of EventType values that trigger the breakpoint.
     * 
     * @return The unique ID of the newly created breakpoint.
     */
    size_t addEventBreakpoint(const std::vector<EventType>& triggers);

    /**
     * @brief Remove a breakpoint by its unique ID.
     *
     * @param breakpointID Breakpoint ID to remove.
     * 
     * @return True if a breakpoint was removed, false if no such ID exists.
     */
    bool removeBreakpoint(size_t breakpointID);

    /**
     * @brief Remove all breakpoints.
     */
    void clearAllBreakpoints();

    /**
     * @brief Evaluate whether the simulation should pause at the current tick.
     *
     * This function checks all registered breakpoints against the current
     * simulation state and tick. It supports both:
     * 
     * - Tick-based breakpoints (triggered when the current tick matches)
     * - Robot state-based breakpoints (triggered when a robot enters a specific mode)
     *
     * @param state The current simulation state snapshot.
     * @param tick The current simulation tick.
     *
     * @return True if any breakpoint condition is satisfied and execution should pause, false otherwise.
     */
    bool shouldBreak(const SimulationState& state, size_t tick) const;

    /**
     * @brief Retrieve all stored breakpoints.
     *
     * @return Const reference to the vector of breakpoints.
     */
    const std::vector<std::unique_ptr<Breakpoint>>& getBreakpoints() const;
};