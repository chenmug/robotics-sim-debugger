#pragma once
#include "core/SimulationState.hpp"  // Forward Declaration
#include <vector>                    // For std::vector
#include <cstddef>                   // For size_t

/**
 * @brief Manages snapshots of the simulation state for time-travel debugging.
 *
 * SnapshotManager stores a history of SimulationState objects, each representing
 * the world state at a specific simulation tick.
 *
 * The main purpose of this class is to enable advanced debugging capabilities
 * such as:
 *
 * - stepping forward through the simulation
 * - stepping backward to a previous tick
 * - replaying past states
 * - timeline navigation in a GUI debugger
 *
 * Snapshots are stored in chronological order and can be accessed by their
 * corresponding simulation tick index.
 *
 * The class does not modify simulation states. It only stores copies of them
 * as they are produced by the SimulationEngine.
 */
class SnapshotManager
{
private:

    /**
     * @brief Stored simulation snapshots.
     *
     * Each entry represents the full SimulationState at a specific tick.
     * The index in this vector corresponds to the simulation tick number
     * when the snapshot was taken.
     */
    std::vector<SimulationState> snapshots_;

public:

    /**
     * @brief Construct a SnapshotManager with optional reserved capacity.
     *
     * Reserving capacity reduces memory reallocations when storing
     * many simulation snapshots.
     *
     * @param capacity Initial capacity for stored snapshots.
     */
    explicit SnapshotManager(size_t capacity = 1000);

    /**
     * @brief Store a snapshot of the current simulation state.
     *
     * The provided state is copied and appended to the snapshot history.
     *
     * @param state Current simulation state to store.
     */
    void save(const SimulationState& state);

    /**
     * @brief Retrieve a snapshot by simulation tick.
     *
     * @param tick Simulation tick index.
     * 
     * @return Pointer to the snapshot if it exists, otherwise nullptr.
     */
    const SimulationState* get(size_t tick) const;

    /**
     * @brief Retrieve the most recent stored snapshot.
     *
     * @return Pointer to the latest snapshot, or nullptr if no snapshots exist.
     */
    const SimulationState* getLast() const;

    /**
     * @brief Get the number of stored snapshots.
     *
     * @return Total number of snapshots currently stored.
     */
    size_t getSize() const noexcept;

    /**
     * @brief Check whether stepping back in time is possible.
     *
     * @param currentTick Current simulation tick.
     * 
     * @return True if a previous snapshot exists, false otherwise.
     */
    bool canStepBack(size_t currentTick) const;

    /**
     * @brief Clear all stored snapshots. Used when resetting the simulation.
     */
    void clearSnapshots();
};