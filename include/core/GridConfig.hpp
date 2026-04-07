#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include <vector>                    // For std::vector


/**
 * @brief Configuration for the simulation grid.
 * 
 * Defines the size of the grid and positions of static obstacles.
 */
struct GridConfig 
{
    int width;                              // Width of the grid
    int height;                             // Height of the grid
    std::vector<Position> static_obstacles; // Positions of obstacles that cannot move

    /**
     * @brief Checks whether a given position lies within the grid boundaries.
     *
     * @param pos The position to validate.
     * 
     * @return True if the position is within grid bounds, false otherwise.
     */
    bool isWithinBounds(const Position& pos) const
    {
        return pos.x >= 0 && pos.x < width && pos.y >= 0 && pos.y < height;
    }

    /**
     * @brief Returns the set of possible movement directions (4-connected grid).
     *
     * @return A constant reference to a vector of direction.
     */
    const std::vector<Position>& getDirections() const
    {
        static const std::vector<Position> directions = {{0,1}, {1,0}, {0,-1}, {-1,0}};

        return directions;
    }
};