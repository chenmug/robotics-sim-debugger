#pragma once
#include <vector>    // For std::vector
#include <optional>  // For std::optional
#include <cstddef>   // For size_t

/**
 * @brief Types of breakpoints supported by the simulation debugger.
 *
 * TICK: Pause execution at a specific simulation tick.
 * ROBOT_MODE: Pause when a robot enters a specific mode/state.
 */
enum class BreakpointType 
{
    TICK,       // Tick-based breakpoint
    ROBOT_MODE  // Robot state-based breakpoint
};

/**
 * @brief Possible operational states of a robot for state-based breakpoints.
 *
 * These modes correspond to stages in the robot's sense-plan-act cycle.
 */
enum class RobotMode 
{
    IDLE,       // Robot is idle
    PLANNING,   // Robot is currently planning a path
    MOVING,     // Robot is moving along its planned path
    REPLANNING  // Robot is replanning its path due to obstacle or update
};

/**
 * @brief Represents a single breakpoint in the simulation.
 *
 * A breakpoint can be either:
 * - Tick-based: triggers when the simulation reaches a specific tick.
 * - Robot state-based: triggers when a specific robot enters a specific RobotMode.
 *
 * Each breakpoint has a unique identifier for removal or management.
 */
struct Breakpoint
{
    size_t breakpointID;            // Unique identifier for this breakpoint
    BreakpointType type;            // Type of breakpoint (TICK or ROBOT_MODE)

    // Tick-based data
    std::optional<size_t> tick;     // Simulation tick (valid if type == TICK)

    // Robot state-based data
    std::optional<size_t> robotId;  // Robot ID (valid if type == ROBOT_MODE)
    std::optional<RobotMode> mode;  // RobotMode (valid if type == ROBOT_MODE)
};

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
    std::vector<Breakpoint> breakpoints_;  // Stored breakpoints
    size_t nextBreakpointID_ = 0;          // Next unique ID for breakpoints

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
     * @brief Check whether any breakpoint is triggered at the current simulation tick.
     *
     * @param currentTick Current simulation tick.
     * 
     * @return True if a tick-based breakpoint, false otherwise.
     */
    bool isTickBreakpoint(size_t currentTick) const;

    /**
     * @brief Check whether any breakpoint is triggered for a robot's state.
     *
     * @param robotId ID of the robot.
     * @param mode Current mode of the robot.
     * 
     * @return True if a robot state breakpoint, false otherwise.
     */
    bool isRobotBreakpoint(size_t robotId, RobotMode mode) const;

    /**
     * @brief Retrieve all stored breakpoints.
     *
     * @return Const reference to the vector of breakpoints.
     */
    const std::vector<Breakpoint>& getBreakpoints() const;
};