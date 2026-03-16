#pragma once
#include "Sensor.hpp"                 // Forward Declaration
#include "core/SimulationEngine.hpp"  // Forward Declaration

/**
 * @brief Sensor that detects obstacles around the robot.
 *
 * This sensor queries the simulation state and returns a SensorData
 * containing information about obstacles near the robot, including
 * their positions and distances.
 */
class ObstacleSensor : public Sensor
{
private:

    int range_;  // Detection range in grid units.

public:

    /**
     * @brief Constructs an ObstacleSensor with a specified detection range.
     *
     * @param range The detection range in grid units. defult is 5.
     */
    explicit ObstacleSensor(int range = 5);

    /**
     * @brief Reads the current simulation state and returns sensor data.
     *
     * @param state The current dynamic state of the simulation (robots and dynamic obstacles)
     * @param grid The grid configuration (size and static obstacles).
     * @param robotID The ID of the robot using the sensor.
     * 
     * @return A SensorData object containing detected obstacles, their distances, and 
     *         the active status of the sensor.
     */
    SensorData read(const SimulationState& state, const GridConfig& grid, size_t robotID) override;

    /**
     * @brief Returns the sensor name.
     *
     * @return The sensor name (e.g., "ObstacleSensor").
     */
    std::string getName() const override;

private:

    /**
     * @brief Adds an obstacle position to the sensor data if the obstacle is within the sensor's range.
     *
     * This function calculates the Euclidean distance between the obstacle and the robot. If the obstacle
     * is within the sensor's range, it is added to the list of detected positions in the `SensorData` object.
     *
     * @param obstaclePos The position of the obstacle to check.
     * @param robotPos The position of the robot (sensor) that detects the obstacle.
     * @param data The sensor data object to which the detected obstacle positions will be added.
     */
    void addObstacleIfInRange(const Position& obstaclePos, const Position& robotPos, SensorData& data);
};