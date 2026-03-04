#pragma once
#include <vector>  // For std::vector

/**
 * @brief Represents a 2D coordinate in the simulation grid.
 */
struct Position 
{
    int x; // X coordinate (column)
    int y; // Y coordinate (row)
};

/**
 * @brief Represents the state of a single robot in the simulation.
 * 
 * Contains its current position, goal, planned path, and index in the path.
 */
struct RobotState 
{
    Position position;                  // Current position of the robot
    Position goal;                      // Target goal position
    std::vector<Position> planned_path; // Precomputed path (absolute coordinates)
    size_t path_index = 0;              // Index of the next step in the planned path
};

/**
 * @brief Represents the full state of the simulation at a given tick.
 * 
 * Includes the tick counter, all robots' states, and dynamic obstacles in the grid.
 */
struct SimulationState 
{
    size_t tick = 0;                         // Current simulation tick
    std::vector<RobotState> robots;          // List of all robots in the simulation
    std::vector<Position> dynamic_obstacles; // Positions of moving obstacles
};