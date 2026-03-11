#pragma once
#include <vector>   // For std::vector
#include <cstddef>  // For size_t

/**
 * @brief Represents a 2D coordinate in the simulation grid.
 *
 * The grid uses integer coordinates where:
 * - x represents the column
 * - y represents the row
 */
struct Position 
{
    int x; // X coordinate (column)
    int y; // Y coordinate (row)

    // Default constructor
    Position() = default;

    // Constructs a position with given coordinates
    Position(int x_, int y_) : x(x_), y(y_) {}

    // Equality comparison for positions.
    bool operator==(const Position& other) const
    {
        return x == other.x && y == other.y;
    }

    // Inequality comparison between two positions.
    bool operator!=(const Position& other) const
    {
        return !(*this == other);
    }
};

/**
 * @brief Represents the state of a single robot in the simulation.
 *
 * RobotState is a plain data structure describing the dynamic state
 * of a robot at a specific simulation tick.
 *
 * It stores:
 * - the robot's unique identifier
 * - its current position
 * - its navigation goal
 * - the planned path produced by a planner
 * - the index of the next step along that path
 *
 * The simulation engine updates this structure each tick as the robot
 * moves through the environment.
 */
struct RobotState 
{
    size_t id;                          // Unique robot identifier
    Position position;                  // Current position of the robot
    Position goal;                      // Target goal position
    std::vector<Position> planned_path; // Precomputed path (absolute coordinates)
    size_t path_index = 0;              // Index of the next step in the planned path
};

/**
 * @brief Represents the complete state of the simulation at a given tick.
 *
 * SimulationState captures all dynamic information required to describe
 * the world at a specific point in time. It is designed to be easily
 * copied and stored by the SnapshotManager to enable time-travel
 * debugging (rewind / step-back functionality).
 *
 * The state includes:
 * - the current simulation tick
 * - the state of all robots
 * - positions of dynamic obstacles
 */
struct SimulationState 
{
    size_t tick = 0;                         // Current simulation tick
    std::vector<RobotState> robots;          // List of all robots in the simulation
    std::vector<Position> dynamic_obstacles; // Positions of moving obstacles
};