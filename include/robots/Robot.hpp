#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include "sensors/Sensor.hpp"        // Forward Declaration
#include "robots/RobotMode.hpp"      // Forward Declaration
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
 * relevant parts of it during execution.
 */
class Robot
{
protected:
    size_t id_ = 0;                                 // Unique robot identifier
    std::vector<std::shared_ptr<Sensor>> sensors_;  // List of sensors attached to this robot
    std::vector<SensorData> sensorDataCache_;       // Cached readings from all sensors
    RobotMode mode_ = RobotMode::IDLE;              // Current mode of the robot

public:
    /**
     * @brief Set the unique identifier to the robot
     *  
     * Each robot in the simulation should have a distinct ID.  
     *
     * @param id a unique robot identifier
     */
    void setID(size_t id);

    /**
     * @brief Get the unique identifier of the robot.
     * 
     * This ID uniquely identifies the robot within the simulation.
     * 
     * @return size_t The robot's unique ID
     */
    size_t getID() const;

    /**
     * @brief Attach a new sensor to this robot.
     *
     * @param sensor Shared pointer to a sensor instance.
     */
    void addSensor(std::shared_ptr<Sensor> sensor);

    /**
     * @brief Access the last cached sensor readings.
     *
     * @return const reference to vector of SensorData
     */
    const std::vector<SensorData>& getSensorDataCache() const;

    /**
     * @brief Set the current mode of the robot.
     */
    void setMode(RobotMode mode);

    /**
     * @brief Get the current mode of the robot.
     */
    RobotMode getMode() const;

    /**
     * @brief Observe the environment.
     *
     * @param state Current simulation state (read-only).
     */
    virtual void sense(const SimulationState& state) = 0;

    /**
     * @brief Plan the next actions.
     *
     * Computes the robot's next intended move based on the
     * current simulation state.
     *
     * The robot may update its own planned fields in the
     * SimulationState (e.g., nextPlannedPos) to announce its
     * intention and allow other robots to avoid collisions.
     *
     * @param state Current simulation state.
     */
    virtual void plan(SimulationState& state) = 0;

    /**
     * @brief Execute the planned action.
     *
     * This function applies the robot's planned action to the
     * simulation state (e.g., updating position).
     *
     * @param state Current simulation.
     */
    virtual void act(SimulationState& state) = 0;

    /**
     * @brief Synchronize the robot's internal caches with the SimulationState.
     *
     * Must update all relevant fields including:
     * - current position
     * - goal position
     * - planned path caches
     * - next planned position
     * - mode (crucial for robot-based breakpoints)
     *
     * @param state The current global simulation state.
     */
    virtual void syncWithState(const SimulationState& state) = 0;

    // Virtual destructor for proper cleanup of derived classes
    virtual ~Robot() = default;
};