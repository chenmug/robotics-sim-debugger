#pragma once
#include "BenchmarkResult.hpp"  // Forward Declaration
#include <vector>               // For std::vector

/**
 * @brief Prints aggregated benchmark statistics across all configurations.
 *
 * Computes and displays the average performance improvement of A*
 * compared to BFS and Dijkstra in terms of node expansions.
 *
 * @param results Vector of benchmark results for all grid configurations.
 */
void printFinalSummary(const std::vector<BenchmarkResult>& results);