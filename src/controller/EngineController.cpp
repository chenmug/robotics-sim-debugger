#include "controller/EngineController.hpp"
#include <thread>
#include <chrono>
#include <iostream>



// TODO: move to GUI layer when switching to graphical rendering
void printGrid(const SimulationState& state, const GridConfig& grid)
{
    std::system("clear"); 
    std::cout << "Robotics Simulation Debugger - Console MVP\n";
    std::cout << "Tick: " << state.tick << "\n\n";

    for (int y = 0; y < grid.height; ++y)
    {
        for (int x = 0; x < grid.width; ++x)
        {
            Position pos{x, y};
            bool printed = false;

            // Robots
            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                const auto& robot = state.robots[i];
                if (robot.position == pos)
                {
                    std::cout << "R" << i + 1 << " ";
                    printed = true;
                    break;
                }
            }
            if (printed) continue;

            // Static obstacles
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

            // Dynamic obstacles
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

            // Goals
            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                const auto& robot = state.robots[i];
                if (robot.goal == pos)
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
    // Ensure simulation thread exits safely before destruction
    quit();

    if (simulationThread_.joinable())
    {
        simulationThread_.join();
    }
}


// /************ GET CURRENT TICK ***************/

size_t EngineController::getCurrentTick() const 
{ 
    return currentTick_; 
}


// /*************** SIMULATION LOOP *************/

void EngineController::simulationLoop()
{
    std::unique_lock<std::mutex> lock(mtx_);

    while (true)
    {
        // Wait until running or quitting
        cv_.wait(lock, [this]() { return isRunning_ || quitRequested_; });

        if (quitRequested_)
            break;

        if (engine_.allRobotsReached())
        {
            isRunning_ = false;
            cv_.notify_all(); 
            continue; 
        }

        engine_.runTick();
        ++currentTick_;
        updateGUI();

        if (checkBreakpoints())
        {
            isRunning_ = false;
            std::cout << "[Simulation paused due to breakpoint at tick " << currentTick_ << "]\n";
        }

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
        if (!isRunning_)
        {
            isRunning_ = true;
            quitRequested_ = false;
        }
    }

    cv_.notify_all();
}


// /****************** PAUSE ******************/

void EngineController::pause()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        isRunning_ = false;

        currentTick_ = snapshot_.getSize() - 1;
    }

    cv_.notify_all();
}


// /************** STEP FORWARD ***************/

void EngineController::stepForward()
{
    std::unique_lock<std::mutex> lock(mtx_);
    isRunning_ = false;

    if (currentTick_ + 1 < snapshot_.getSize())
    {
        ++currentTick_;
        engine_.setCurrentState(*snapshot_.get(currentTick_));
    }
    else
    {
        engine_.runTick();
        ++currentTick_;
    }

    updateGUI();
}


// /**************** STEP BACK *****************/

void EngineController::stepBack()
{
    std::unique_lock<std::mutex> lock(mtx_);
    isRunning_ = false;

    if (currentTick_ > 0)
    {
        --currentTick_;
        const SimulationState* state = snapshot_.get(currentTick_);
        if (state)
        {
            engine_.setCurrentState(*state);
        }
    }

    updateGUI();
}


// /**************** UPDATE GUI ****************/

void EngineController::updateGUI()
{
    const SimulationState* state = nullptr;

    if (isRunning_)  // Run mode
    {
        state = snapshot_.getLast(); 
    }
    else  // Pause mode
    {
        state = snapshot_.get(currentTick_); 
    }

    if (state)
    {
        printGrid(*state, engine_.getGridConfig());
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
    const SimulationState* state = nullptr;

    if (isRunning_)
    {
        state = snapshot_.getLast();
    }
    else
    {
        state = snapshot_.get(currentTick_);
    }

    if (!state)
    {
        return false;
    }

    return breakpointManager_.shouldBreak(*state, currentTick_);
}


// /********* GET BREAKPOINT MANAGER **********/

BreakpointManager& EngineController::getBreakpointManager()
{
    return breakpointManager_;
}