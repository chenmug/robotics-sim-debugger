#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include "core/GridConfig.hpp"       // Forward Declaration
#include <vector>                    // For std::vector
#include <string>                    // For std::string


/**
 * @brief Read-only snapshot used for debugging and UI rendering.
 *
 * Represents a single point-in-time view of the simulation state,
 * consumed by the UI layer without modifying the engine state.
 */
struct DebugSnapshotView
{
    size_t tick = 0;                        // Current simulation tick
    const SimulationState* state = nullptr; // Pointer to simulation state
    const GridConfig* grid = nullptr;       // Pointer to grid configuration
    std::vector<std::string> events;        // Events generated at this tick
    bool isRunning = false;                 // Indicates whether simulation is currently running 
};