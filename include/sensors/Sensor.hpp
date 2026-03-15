#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include <vector>                    // For std::vector
#include <string>                    // For std::string
#include <memory>                    

struct GridConfig;

/**
* @brief Represents the output data from a sensor.
 * 
 * Contains all relevant information detected by the sensor in the environment.
 */
struct SensorData
{
    std::vector<Position> positions;  // Positions of detected objects (obstacles or goals)
    std::vector<double> distances;    // Optional distances to detected objects
    bool active = true;               // Whether the sensor is currently active
};

/**
 * @brief Abstract base class for all sensors.
 *
 * Sensors read the simulation state and return SensorData. Each sensor must provide a
 * unique name for GUI identification.
 */
class Sensor
{
public:

    /**
     * @brief Reads the current simulation state and returns sensor data.
     *
     * @param state The current dynamic state of the simulation (robots and dynamic obstacles)
     * @param grid The grid configuration (size and static obstacles).
     * @param robotID The ID of the robot using the sensor.
     * 
     * @return SensorData containing the positions detected by the sensor.
     */
    virtual SensorData read(const SimulationState& state, const GridConfig& grid, size_t robotID) = 0;

    /**
     * @brief Returns the sensor name for GUI display.
     *
     * @return std::string Sensor name (e.g., "GoalSensor", "ObstacleSensor")
     */
    virtual std::string getName() const = 0;

    // Virtual destructor for proper cleanup of derived sensor classes
    virtual ~Sensor() = default;
};