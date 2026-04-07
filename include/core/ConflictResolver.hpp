#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include <vector>                    // For std::vector

struct GridConfig;                   // Forward Declaration

/**
 * @class ConflictResolver
 * @brief Handles conflict resolution between robots in the simulation.
 *
 * This class is responsible for detecting and resolving movement conflicts
 * between multiple robots attempting to act within the same simulation tick.
 *
 * The resolver enforces deterministic behavior by applying a fixed policy:
 * - First conflict between two robots -> both wait for one tick
 * - Repeated conflict -> priority-based resolution (lower ID wins)
 * - Lower-priority robot may be forced to move aside
 */
class ConflictResolver
{
public:
    /**
     * @brief Resolve conflicts for the current simulation tick.
     *
     * This function inspects all robots' planned moves (nextPlannedPos)
     * and modifies them if necessary to prevent:
     * - Multiple robots entering the same cell
     * - Swap collisions (robots exchanging positions)
     * - Deadlocks (robots stuck in repeated blocking states)
     *
     * @param state The full simulation state (modified in-place)
     * @param grid  The grid configuration
     */
    void resolve(SimulationState& state, const GridConfig& grid);

private:
    /**
     * @brief Find a free neighboring position for a robot.
     *
     * Searches adjacent cells (up/down/left/right) and returns the first
     * valid position that:
     * - Is within grid bounds
     * - Is not currently occupied by another robot
     *
     * If no free position is found, the robot remains in its current position.
     *
     * @param state The current simulation state
     * @param robot The robot attempting to move aside
     * @param grid  The grid configuration
     * 
     * @return A valid position for movement (or current position if none found)
     */
    Position findFreeNeighbor(const SimulationState& state, const RobotState& robot, 
            const GridConfig& grid) const;

    /**
     * @brief Reset the 'blockedNow' state for all robots.
     *
     * Sets the 'blockedNow' flag of all robots in the simulation state to false.
     * This function is called at the start of each simulation tick to prepare for conflict resolution.
     *
     * @param state The current simulation state
     */
    void resetBlockedNow(SimulationState& state);

    /**
     * @brief Print the status of all robots for debugging purposes.
     *
     * Prints the current position, next planned position, and blocked state of each robot
     * to the console for debugging.
     *
     * @param state The current simulation state
     */
    void printRobotStatus(const SimulationState& state);

    /**
     * @brief Detect and handle conflicts between all pairs of robots.
     *
     * Checks all pairs of robots to detect conflicts based on their planned movements.
     * If a conflict is detected, the conflict is resolved according to predefined rules:
     * - First-time conflict: both robots wait for one tick.
     * - Repeated conflict: lower-ID robot moves aside.
     *
     * @param state The current simulation state
     * @param grid  The grid configuration
     */
    void detectAndHandleConflicts(SimulationState& state, const GridConfig& grid);

    /**
     * @brief Handle a specific conflict between two robots.
     *
     * Resolves the conflict between two robots by either making them wait or moving
     * the lower-priority robot aside, based on the conflict type and the robots' previous states.
     *
     * @param r1            The first robot involved in the conflict
     * @param r2            The second robot involved in the conflict
     * @param state         The current simulation state
     * @param grid          The grid configuration
     * @param i             The index of the first robot
     * @param j             The index of the second robot
     * @param stuckConflict Flag indicating if the conflict is a "stuck" deadlock situation
     */
    void handleConflict(RobotState& r1, RobotState& r2, SimulationState& state, 
        const GridConfig& grid, size_t i, size_t j, bool stuckConflict);

    /**
     * @brief Update the 'wasBlocked' status for all robots.
     *
     * After resolving conflicts, updates the 'wasBlocked' flag of all robots
     * to reflect whether they were blocked during the current simulation tick.
     *
     * @param state The current simulation state
     */
    void updateBlockedStates(SimulationState& state);

    /**
     * @brief Move a low-priority robot aside to a free neighboring cell.
     *
     * When a conflict occurs, the lower-priority robot is moved to a neighboring
     * free cell to resolve the conflict and allow the higher-priority robot to proceed.
     *
     * @param state The current simulation state
     * @param robot The robot attempting to move aside
     * @param grid  The grid configuration
     */
    void moveAside(SimulationState& state, RobotState& robot, const GridConfig& grid);
};