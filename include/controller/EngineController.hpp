#pragma once
#include "core/SimulationEngine.hpp"         // Forward Declaration
#include "core/SnapshotManager.hpp"          // Forward Declaration
#include "controller/BreakpointManager.hpp"  // Forward Declaration
#include "ui/debug/DebugSnapshotView.hpp"    // Forward Declaration
#include <atomic>                            // For std::atomic
#include <thread>                            // For std::thread
#include <mutex>                             // For std::mutex
#include <condition_variable>                // For std::condition_variable


/**
 * @brief Controls the simulation execution and user interactions.
 *
 * EngineController is responsible for:
 * - Coordinating between the SimulationEngine and SnapshotManager.
 * - Managing the simulation loop in a background thread.
 * - Supporting run, pause, step-forward, and step-back operations.
 * - Handling breakpoints (tick-based and robot-mode-based) via BreakpointManager.
 */
class EngineController 
{
private:

    SimulationEngine& engine_;    // The simulation engine
    SnapshotManager& snapshot_;   // The snapshot manager (access via EngineController)

    std::atomic<bool> isRunning_ = false;      // Indicates whether the simulation is currently advancing automatically.
    std::atomic<bool> quitRequested_ = false;  // Signals the simulation thread to terminate.
    std::thread simulationThread_;             // Background thread responsible for advancing simulation ticks.

    mutable std::mutex mtx_;      // Mutex for synchronizing access to shared resources.
    std::condition_variable cv_;  // Condition variable for thread synchronization.

    BreakpointManager breakpointManager_;  // Manages breakpoints for tick and robot mode.
    bool isStepping_ = false;              // Indicates whether the simulation is currently stepping.

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

    /**
     * @brief Access the internal BreakpointManager.
     *
     * @return Reference to BreakpointManager.
     */
    BreakpointManager& getBreakpointManager();

    /**
     * @brief Jump immediately to a specific tick in the simulation.
     *
     * This function attempts to move the simulation to the specified tick (`targetTick`).
     * If a snapshot for the target tick exists, it will load that snapshot directly.
     * If no snapshot exists for the given tick, the simulation will advance forward 
     * from the current tick until the target tick is reached, storing snapshots at each step.
     *
     * @param targetTick The tick to jump to.
     * 
     * @return True if the jump was successful (either by loading a snapshot or advancing the 
     *         simulation), false if the targetTick is out of range and cannot be reached (either due
     *         to snapshot constraints or simulation limits).       
     */
    bool jumpToTick(size_t targetTick);

private:

    // Main simulation loop executed by `simulationThread_`.
    void simulationLoop();

    /**
     * @brief Check if any breakpoints are triggered at the current tick/state.
     *
     * @return True if a breakpoint condition is met, false otherwise.
     */
    bool checkBreakpoints() const;

    /**
     * @brief Synchronizes the simulation engine to a specific tick.
     *
     * This function updates the engine's current state to match the snapshot
     * corresponding to the given tick. It ensures that the simulation state,
     * including all robots and dynamic obstacles, reflects exactly the snapshot
     * at that moment in time.
     *
     * @param tick The target simulation tick to synchronize to.
     */
    void syncToTick(size_t tick);

    /**
     * @brief Builds a snapshot view of the current simulation state for debugging/UI rendering.
     *
     * This function extracts relevant runtime information from the SimulationEngine
     * and packages it into a DebugSnapshotView structure, which is used by the UI layer
     * (console or future graphical interface) to render the current simulation state.
     *
     * The view includes:
     * - Current simulation tick
     * - Simulation state
     * - Grid configuration reference
     * - Event system
     *
     * @return DebugSnapshotView containing a structured snapshot of the current simulation state.
     */
    DebugSnapshotView buildDebugView();
};