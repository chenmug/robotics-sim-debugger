#include "controller/Breakpoint.hpp"  // Forward Declaration
#include "robots/Robot.hpp"           // Forward Declaration


/**
 * @brief Breakpoint that triggers when a specific robot enters a given mode.
 *
 * This breakpoint pauses the simulation when the robot identified by
 * `robotId` reaches the specified `mode`.
 */
class RobotModeBreakpoint : public Breakpoint
{
private:

    size_t robotId_;  // ID of the robot to monitor
    RobotMode mode_;  // The robot mode that triggers the breakpoint

public:
    /**
     * @brief Construct a new RobotModeBreakpoint.
     *
     * @param id Unique identifier for this breakpoint instance.
     * @param robotId The ID of the robot to monitor.
     * @param mode The mode that will trigger the breakpoint.
     */
    RobotModeBreakpoint(size_t id, size_t robotId, RobotMode mode);

    /**
     * @brief Check if the breakpoint condition is met.
     *
     * @param state Current simulation state.
     * @param tick Current simulation tick (unused in this breakpoint type).
     * 
     * @return True if the monitored robot is in the specified mode, false otherwise.
     *
     * @note The `tick` parameter is marked [[maybe_unused]] since this
     *       breakpoint depends only on the robot's mode.
     */
    bool shouldBreak([[maybe_unused]] const SimulationState& state, [[maybe_unused]] size_t tick) const override;
};