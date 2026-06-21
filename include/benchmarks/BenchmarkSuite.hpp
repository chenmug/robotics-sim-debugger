#pragma once
#include "benchmarks/BenchmarkConfig.hpp"  // Forward Declaration
#include <vector>                          // For std::vector                      

/**
 * @brief Creates a predefined set of benchmark scenarios.
 *
 * Each scenario defines grid size, obstacle density, and repetition count
 * used to evaluate pathfinding algorithms under controlled conditions.
 */
std::vector<BenchmarkConfig> createBenchmarkSuite();