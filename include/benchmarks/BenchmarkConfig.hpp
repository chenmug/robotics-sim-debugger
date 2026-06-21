#pragma once

/**
 * @brief Configuration for a single benchmarking scenario.
 *
 * Defines the parameters of a grid-based experiment used to evaluate
 * and compare path-planning algorithms (BFS, Dijkstra, A*).
 *
 * Each configuration is executed multiple times to compute averaged
 * performance metrics under deterministic conditions.
 */
struct BenchmarkConfig
{
    int width;               // Grid width 
    int height;              // Grid height
    double obstacleDensity;  // Percentage of blocked cells (0.0 - 1.0)
    int runsPerConfig;       // Number of repetitions for averaging results
};