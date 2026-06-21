#pragma once
#include "BenchmarkConfig.hpp"  // Forward Declaration
#include "BenchmarkResult.hpp"  // Forward Declaration

/**
 * @brief Runs a full benchmark for BFS, Dijkstra, and A* on a single grid configuration.
 *
 * Executes multiple runs on randomly generated grids according to the given configuration,
 * and returns averaged performance metrics for each algorithm.
 *
 * @param config Benchmark parameters (grid size, obstacle density, number of runs).
 * @return Aggregated benchmark results for all algorithms.
 */
BenchmarkResult runBenchmark(const BenchmarkConfig& config);