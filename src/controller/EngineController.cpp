#include "controller/EngineController.hpp"
#include "ui/console/ConsoleRenderer.hpp"
#include "robots/GridRobot.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <string>


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
    DebugSnapshotView view = buildDebugView();
    renderView(view);
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


// /************ BUILD DEBUG VIEW ***************/

DebugSnapshotView EngineController::buildDebugView()
{
    DebugSnapshotView view;
    size_t tick = engine_.getCurrentState().tick;

    const SimulationState* state = snapshot_.get(tick);
    if (!state)
    {
        return view; 
    }

    view.tick = state->tick;
    view.state = state;  
    view.grid = &engine_.getGridConfig();
    view.selectedRobotId = selectedRobotId_;
    view.isRunning = isRunning_;
    view.robotsInfo = collectRobotDebugInfo();

    return view;
}


// /******** COLLECT PLANNER DEBUG INFO *********/

std::vector<RobotDebugInfo> EngineController::collectRobotDebugInfo() const
{
    std::vector<RobotDebugInfo> result;

    for (const auto& robot : engine_.getRobots())
    {
        RobotDebugInfo info;

        info.name = "R" + std::to_string(robot->getID() + 1);

        if (auto* planner = robot->getPlanner())
        {
            info.algorithm = planner->getAlgorithmName();
            info.nodesExpanded = planner->getNodesExpanded();
            info.plannerTimeMs = planner->getLastRunTimeMs();
        }

        if (auto* gridRobot = dynamic_cast<const GridRobot*>(robot.get()))
        {
            info.pathLength = gridRobot->getPathLength();
            const auto& fullPath = gridRobot->getPath();
            size_t idx = gridRobot->getPathIndex();

            if (idx > fullPath.size())
            {
                idx = fullPath.size();
            }
            info.path.assign(fullPath.begin(), fullPath.begin() + idx);
        }

        result.push_back(info);
    }

    return result;
}


// /************ SET SELECTED ROBOT *************/

void EngineController::setSelectedRobot(int id)
{
    std::lock_guard<std::mutex> lock(mtx_);
    selectedRobotId_ = id;
    updateGUI();
}


// /************ GET SELECTED ROBOT *************/

int EngineController::getSelectedRobot() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return selectedRobotId_;
}