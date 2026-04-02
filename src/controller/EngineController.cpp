#include "controller/EngineController.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <string>


// TODO: move to GUI layer when switching to graphical rendering
// Helper: Converts an EventType enum to a human-readable string
inline std::string eventTypeToString(EventType type)
{
    switch(type)
    {
        case EventType::OBSTACLE_DETECTED: 
            return "ObstacleDetected";

        case EventType::REPLAN_TRIGGERED:  
            return "ReplanTriggered";

        case EventType::COLLISION_DETECTED: 
            return "CollisionDetected";

        case EventType::GOAL_REACHED:
            return "GoalReached";

        default: return "UnknownEvent";
    }
}


// TODO: move to GUI layer when switching to graphical rendering
void printGrid(const SimulationState& state, const GridConfig& grid, size_t tick)
{
    // std::system("clear"); 
    std::cout << "Robotics Simulation Debugger - Console MVP\n";
    std::cout << "Tick: " << tick << "\n\n";

    for (int y = 0; y < grid.height; ++y)
    {
        for (int x = 0; x < grid.width; ++x)
        {
            Position pos{x, y};
            bool printed = false;

            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                if (state.robots[i].position == pos)
                {
                    std::cout << "R" << i + 1 << " ";
                    printed = true;
                    break;
                }
            }

            if (printed) continue;

            for (const auto& obst : grid.static_obstacles)
            {
                if (obst == pos)
                {
                    std::cout << "X ";
                    printed = true;
                    break;
                }
            }

            if (printed) continue;

            for (const auto& dyn : state.dynamic_obstacles)
            {
                if (dyn == pos)
                {
                    std::cout << "X ";
                    printed = true;
                    break;
                }
            }

            if (printed) continue;

            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                if (state.robots[i].goal == pos)
                {
                    std::cout << "G" << i + 1 << " ";
                    printed = true;
                    break;
                }
            }

            if (printed) continue;

            std::cout << ". ";
        }

        std::cout << "\n";
    }

    std::cout << "\n";

    // Print events for this tick
    if (!state.events.empty())
    {
        std::cout << "Events:\n";
        for (const auto& event : state.events)
        {
            std::cout << " [Event] Robot " << event.robotId + 1
                      << " triggered " << eventTypeToString(event.type) 
                      << " at tick " << event.tick << "\n";
        }
        std::cout << "\n";
    }
}







// /*************** CONSTRUCTOR *****************/

EngineController::EngineController(SimulationEngine& engine, SnapshotManager& snapshotManager)
    : engine_(engine), snapshot_(snapshotManager)
{
    simulationThread_ = std::thread(&EngineController::simulationLoop, this);
}


// /*************** DESTRUCTOR ******************/

EngineController::~EngineController()
{
    quit();

    if (simulationThread_.joinable())
    {
        simulationThread_.join();
    }
}


// /************ GET CURRENT TICK ***************/

size_t EngineController::getCurrentTick() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return engine_.getCurrentState().tick;
}


// /*************** SIMULATION LOOP *************/

void EngineController::simulationLoop()
{
    std::unique_lock<std::mutex> lock(mtx_);

    while (true)
    {
        cv_.wait(lock, [this]() { return isRunning_ || quitRequested_; });

        if (quitRequested_)
        {
            break;
        }

        isStepping_ = true;

        lock.unlock();
        engine_.runTick();
        lock.lock();

        syncToTick(engine_.getCurrentState().tick);

        if (engine_.allRobotsReached())
        {
            isRunning_ = false;
            quitRequested_ = true;
            cv_.notify_all();

            break;
        }

        if (checkBreakpoints())
        {
            isRunning_ = false;
            std::cout << "[Simulation paused due to breakpoint at tick " << engine_.getCurrentState().tick << "]\n";
        }

        isStepping_ = false;
        cv_.notify_all();

        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        lock.lock();
    }
}


// /******************** RUN ********************/

void EngineController::run()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);

        size_t currentTick = engine_.getCurrentState().tick;
        if (currentTick + 1 < snapshot_.getSize())
        {
            snapshot_.removeFutureSnapshots(currentTick + 1);
        }

        isRunning_ = true;
        quitRequested_ = false;
    }

    cv_.notify_all();
}


// /****************** PAUSE ******************/

void EngineController::pause()
{
    std::lock_guard<std::mutex> lock(mtx_);
    isRunning_ = false;

    size_t lastTick = snapshot_.getSize() - 1;
    syncToTick(lastTick);
}


// /************** STEP FORWARD ***************/

void EngineController::stepForward()
{
    std::unique_lock<std::mutex> lock(mtx_);
    isRunning_ = false;

    size_t nextTick = engine_.getCurrentState().tick + 1;
    if (nextTick < snapshot_.getSize())
    {
        syncToTick(nextTick);
    }
    else
    {
        lock.unlock();
        engine_.runTick();
        lock.lock();

        syncToTick(nextTick);
    }

    cv_.notify_all();
}


// /**************** STEP BACK *****************/

void EngineController::stepBack()
{
    std::unique_lock<std::mutex> lock(mtx_);
    isRunning_ = false;

    size_t currentTick = engine_.getCurrentState().tick;

    if (currentTick == 0)
    {
        return;
    }

    size_t newTick = currentTick - 1;
    syncToTick(newTick);

    snapshot_.removeFutureSnapshots(newTick + 1);

    cv_.notify_all();
}


// /**************** UPDATE GUI ****************/

void EngineController::updateGUI()
{
    size_t tick = engine_.getCurrentState().tick;

    const SimulationState* state = snapshot_.get(tick);
    if (state)
    {
        printGrid(*state, engine_.getGridConfig(), tick);
    }
}


// /****************** QUIT *******************/

void EngineController::quit()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        isRunning_ = false;
        quitRequested_ = true;
    }

    cv_.notify_all();
}


// /*************** IS FINISHED ***************/

bool EngineController::isFinished() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return engine_.allRobotsReached();
}


// /************ CHECK BREAKPOINTS ************/

bool EngineController::checkBreakpoints() const
{
    size_t tick = engine_.getCurrentState().tick;

    const SimulationState* state = snapshot_.get(tick);
    if (!state)
    {
        return false;
    }

    return breakpointManager_.shouldBreak(*state, tick);
}


// /********* GET BREAKPOINT MANAGER **********/

BreakpointManager& EngineController::getBreakpointManager()
{
    return breakpointManager_;
}


// /************** SYNC TO TICK ***************/

void EngineController::syncToTick(size_t tick)
{
    const SimulationState* state = snapshot_.get(tick);
    if (!state) 
    {
        return;
    }

    SimulationState synced = *state;
    synced.tick = tick;

    engine_.setCurrentState(synced);

    updateGUI();
}


// /************** JUMP TO TICK ***************/

bool EngineController::jumpToTick(size_t targetTick)
{
    std::unique_lock<std::mutex> lock(mtx_);

    isRunning_ = false;
    cv_.notify_all();

    cv_.wait(lock, [this]() { return !isStepping_; });

    if (targetTick >= snapshot_.getCapacity())
    {
        return false;
    }

    if (targetTick < snapshot_.getSize())
    {
        syncToTick(targetTick);
        snapshot_.removeFutureSnapshots(targetTick + 1);

        return true;
    }

    while (engine_.getCurrentState().tick < targetTick)
    {
        lock.unlock();
        engine_.runTick();
        lock.lock();

        size_t tick = engine_.getCurrentState().tick;

        const SimulationState* state = snapshot_.get(tick);
        if (!state)
        {
            snapshot_.save(engine_.getCurrentState());
        }

        syncToTick(tick);

        if (checkBreakpoints())
        {
            return false;
        }
    }

    syncToTick(engine_.getCurrentState().tick);

    return true;
}