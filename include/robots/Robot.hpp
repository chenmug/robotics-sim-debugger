#pragma once
#include "SimulationState.hpp"

/**
 * @brief Abstract base class for all robots in the simulation.
 * 
 * Defines the core interface for sensing the environment, planning actions,
 * and acting on the simulation state. Each concrete robot must implement these methods.
 * 
 * The typical tick sequence for a robot is:
 *   1. sense() - observe the environment
 *   2. plan() - decide next moves or path
 *   3. act() - execute actions and update the SimulationState
 */
class Robot 
{
public:
    /**
     * @brief Sense the environment based on the current SimulationState.
     * 
     * @param state The current simulation state (read-only).
     *              The robot can use this information to update its internal sensors or perception.
     */
    virtual void sense(const SimulationState& state) = 0;

    /**
     * @brief Compute the robot's next moves or path.
     * 
     * @param state The current simulation state (read-write).
     *              The robot examines the environment to decide its next actions:
     *                - Detect obstacles.
     *                - Check positions of other robots.
     *                - Plan its path or next steps.
     *              This function **only plans** and does not change the robot's position or state yet.
     */
    virtual void plan(SimulationState& state) = 0;

    /**
     * @brief Execute the planned actions on the simulation state.
     * 
     * @param state The current simulation state (read-write).
     *              The robot applies the plan computed in plan():
     *                - Updates its position.
     *                - Updates internal state if needed.
     *              This function modifies the simulation state according to the planned actions.
     */
    virtual void act(SimulationState& state) = 0;

    // Virtual destructor for proper cleanup of derived classes
    virtual ~Robot() = default;
};