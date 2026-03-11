#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include <cstddef>                   // For size_t

/**
 * @brief Abstract base class for all robots in the simulation.
 *
 * Robots implement a Sense -> Plan -> Act pipeline:
 *
 *   1. sense() - observe the current world state.
 *   2. plan() - compute the next actions.
 *   3. act() - apply the planned action to the simulation.
 *
 * The Robot does not own the SimulationState; it only reads from it or updates
 *  relevant parts of it during execution.
 */
class Robot
{
protected:
    size_t id_ = 0;  // Unique robot identifier

public:
    /**
     * @brief Set the unique identifier to the robot
     *  
     * Each robot in the simulation should have a distinct ID.  
     *
     * @param id a unique robot identifier
     */
    void setID(size_t id) { id_ = id; }

    /**
     * @brief Get the unique identifier of the robot.
     * 
     * This ID uniquely identifies the robot within the simulation.
     * 
     * @return size_t The robot's unique ID
     */
    size_t getID() const { return id_; }

    /**
     * @brief Observe the environment.
     *
     * @param state Current simulation state (read-only).
     */
    virtual void sense(const SimulationState& state) = 0;

    /**
     * @brief Plan the next actions.
     *
     * This function computes the next steps based on the
     * current world state but does not modify it.
     *
     * @param state Current simulation state (read-only).
     */
    virtual void plan(const SimulationState& state) = 0;

    /**
     * @brief Execute the planned action.
     *
     * This function applies the robot's planned action to the
     * simulation state (e.g., updating position).
     *
     * @param state Current simulation.
     */
    virtual void act(SimulationState& state) = 0;

    // Virtual destructor for proper cleanup of derived classes
    virtual ~Robot() = default;
};