#pragma once
#include <string>  // For std::string

/**
 * @brief Stores performance metrics for a single algorithm run.
 *
 * Contains node expansion count and execution time.
 */
struct AlgorithmStats
{
    double nodes = 0;   // Number of nodes expanded during search
    double timeMs = 0;  // Execution time in milliseconds
};

/**
 * @brief Holds benchmark results for BFS, Dijkstra, and A* on a single grid configuration.
 *
 * Includes a label describing the test scenario and performance statistics for each algorithm.
 */
struct BenchmarkResult
{
    std::string label;         // String description of the grid size
    std::string obstacleInfo;  // String description of the obstacle density
    AlgorithmStats bfs;        // Performance metrics for BFS
    AlgorithmStats dijkstra;   // Performance metrics for Dijkstra
    AlgorithmStats astar;      // Performance metrics for A*
};