#include "core/SnapshotManager.hpp"


/**************** CONSTRUCTOR ****************/

SnapshotManager::SnapshotManager(size_t capacity)
{
    snapshots_.reserve(capacity);
}


/******************* SAVE *******************/

void SnapshotManager::save(const SimulationState& state)
{
    snapshots_.push_back(state);
}


/******************* GET ********************/

const SimulationState* SnapshotManager::get(size_t tick) const
{
    if (tick >= snapshots_.size())
    {
        return nullptr;
    }

    return &snapshots_[tick];
}


/***************** GET LAST ******************/

const SimulationState* SnapshotManager::getLast() const
{
    if (snapshots_.empty())
    {
        return nullptr;
    }

    return &snapshots_.back();
}


/***************** GET SIZE ******************/

size_t SnapshotManager::getSize() const noexcept
{
    return snapshots_.size();
}


/*************** CAN STEP BACK ****************/

bool SnapshotManager::canStepBack(size_t currentTick) const
{
    return currentTick > 0 && currentTick < snapshots_.size();
}


/*************** CAN STEP BACK ****************/

void SnapshotManager::clearSnapshots()
{
    snapshots_.clear();
}