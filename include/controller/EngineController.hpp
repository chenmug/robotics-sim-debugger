#pragma once
#include "core/SimulationEngine.hpp"  // Forward Declaration
#include "core/SnapshotManager.hpp"   // Forward Declaration
#include <atomic>                     // For std::atomic
#include <thread>                     // For std::thread
#include <mutex>                      // For std::mutex
#include <condition_variable>         // For std::condition_variable


/**
 * @brief Controls simulation execution and user interaction.
 *
 * Coordinates between the user interface, SimulationEngine, and SnapshotManager.
 * Manages the simulation thread and supports run, pause, and time-travel debugging
 * (step forward/backward).
 */
class EngineController 
{
private:

    SimulationEngine& engine_;    // The simulation engine
    SnapshotManager& snapshot_;   // The snapshot manager (access via EngineController)
    size_t currentTick_ = 0;      // The current tick of the simulation

    std::atomic<bool> isRunning_ = false;      // Indicates whether the simulation is currently advancing automatically.
    std::atomic<bool> quitRequested_ = false;  // Signals the simulation thread to terminate.
    std::thread simulationThread_;             // Background thread responsible for advancing simulation ticks.

    std::mutex mtx_;              // Mutex for synchronizing access to shared resources.
    std::condition_variable cv_;  // Condition variable for thread synchronization.

public:
    /**
     * @brief Construct a new EngineController.
     *
     * @param engine Reference to the simulation engine.
     * @param snapshotManager Reference to the snapshot manager used for time-travel debugging.
     */
    EngineController(SimulationEngine& engine, SnapshotManager& snapshotManager);

    /**
     * @brief Destructor.
     *
     * Ensures the simulation thread terminates safely before the controller
     * is destroyed by signaling quitRequested_ and joining the thread.
     */
    ~EngineController();

    /**
     * @brief Returns the current tick count of the simulation.
     *
     * @return The current tick.
     */
    size_t getCurrentTick() const;

    // Sets the isRunning_ flag to true and starts the simulation thread.
    void run();

    // Pause the simulation
    void pause();

    // Step forward one tick
    void stepForward();

    // Step back one tick
    void stepBack();

    // Updates the GUI
    void updateGUI();

    // Quit the simulation
    void quit();

    /**
     * @brief Check if simulation is finished - all the robot reached their goals.
     * 
     * @return True if the simulation finished, false otherwise.
     */
    bool isFinished() const;

private:

    // Main simulation loop executed by `simulationThread_`.
    void simulationLoop();
};