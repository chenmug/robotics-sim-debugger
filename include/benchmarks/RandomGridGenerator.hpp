#pragma once
#include "core/GridConfig.hpp"  // Forward Declaration

/**
 * @brief Generates a random grid with obstacles for benchmarking.
 *
 * Ensures the generated grid contains a valid path from start to goal.
 * Obstacles are placed according to the given density.
 *
 * @param width Grid width
 * @param height Grid height
 * @param obstacleDensity Probability of a cell being an obstacle (0.0–1.0)
 * @return A valid grid configuration with at least one path from start to goal
 */
GridConfig createRandomGrid(int width, int height, double obstacleDensity);