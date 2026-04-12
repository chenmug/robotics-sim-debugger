#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include "core/GridConfig.hpp"       // Forward Declaration
#include <vector>                    // For std::vector
#include <string>                    // For std::string


/**
 * @brief Holds debugging information related to each robot in the simulation.
 *
 * This structure contains performance and state details of a robot during the simulation.
 */
struct RobotDebugInfo
{
    std::string name;             // All the robots
    std::string algorithm;        // BFS, DIJKSTRA, A*
    size_t nodesExpanded = 0;     // Nodes explored
    size_t pathLength = 0;        // Current path length
    double plannerTimeMs = 0.0;   // Duration of planning
};

/**
 * @brief Read-only snapshot used for debugging and UI rendering.
 *
 * Represents a single point-in-time view of the simulation state,
 * consumed by the UI layer without modifying the engine state.
 *
 * This structure provides detailed information about the current simulation tick,
 * including the simulation state, grid configuration, events, and robot debug information.
 */
struct DebugSnapshotView
{
    size_t tick = 0;                         // Current simulation tick
    const SimulationState* state = nullptr;  // Pointer to simulation state
    const GridConfig* grid = nullptr;        // Pointer to grid configuration
    std::vector<std::string> events;         // Events generated at this tick
    std::vector<RobotDebugInfo> robotsInfo;  // Robot debug information for each robot in the simulation
    bool isRunning = false;                  // Indicates whether simulation is currently running 
};