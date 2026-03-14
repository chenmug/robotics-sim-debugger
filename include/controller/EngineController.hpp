#pragma once
#include "core/SimulationEngine.hpp"  // Forward Declaration
#include "core/SnapshotManager.hpp"   // Forward Declaration

class EngineController 
{
private:
    SimulationEngine& engine_;    // The simulation engine
    SnapshotManager& snapshot_;   // The snapshot manager (access via EngineController)
    size_t currentTick_ = 0;      // The current tick of the simulation
    bool isRunning_ = false;      // Flag to check if the simulation is running
    bool quitRequested_ = false;  // Flag to check if we quit the simulation

public:
    // Constructor that receives the Simulation Engine and Snapshot Manager
    EngineController(SimulationEngine& engine, SnapshotManager& snapshotManager);

    // Function to run the simulation (advance to the next tick)
    void run();

    // Pause the simulation
    void pause();

    // Step forward one tick
    void stepForward();

    // Step back one tick
    void stepBack();

    // Updates the GUI (this is a placeholder for now)
    void updateGUI();

    // Quit the simulation
    void quit();

    /**
     * @brief Check if simulation is finished - all the robot reached their goals.
     * 
     * @return True if the simulation finished, false otherwise.
     */
    bool isFinished() const;
};