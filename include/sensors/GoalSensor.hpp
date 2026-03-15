#pragma once
#include "Sensor.hpp"                 // Forward Declaration
#include "core/SimulationEngine.hpp"  // Forward Declaration


/**
 * @brief Sensor that detects the goal position of the robot.
 *
 * This sensor queries the simulation state and returns a SensorData
 * containing the goal position that the robot should reach.
 */
class GoalSensor : public Sensor
{
public:

    /**
     * @brief Reads the current simulation state and returns sensor data.
     *
     * @param state The current dynamic state of the simulation (robots and dynamic obstacles)
     * @param grid The grid configuration (size and static obstacles).
     * @param robotID The ID of the robot using the sensor.
     * 
     * @return A SensorData object containing the detected goal position,
     *         distances, and the active status of the sensor.
     */
    SensorData read(const SimulationState& state, const GridConfig& grid, size_t robotID) override;

    /**
     * @brief Returns the sensor name.
     *
     * @return The sensor name (e.g., "GoalSensor").
     */
    std::string getName() const override;
};