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




/**************** CONSTRUCTOR ****************/

EngineController::EngineController(SimulationEngine& engine, SnapshotManager& snapshotManager)
    : engine_(engine), snapshot_(snapshotManager) {}


/**************** DESTRUCTOR *****************/

EngineController::~EngineController()
{
    quit();
    
    if (simulationThread_.joinable())
    {
        simulationThread_.join();
    }
}


/*************** SIMULATION LOOP *************/

void EngineController::simulationLoop()
{
    while (!quitRequested_)
    {
        if (isRunning_ && !engine_.allRobotsReached()) 
        {
            engine_.runTick();
            ++currentTick_;
            updateGUI();
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
        }
        else
        {
            isRunning_ = false;
            quitRequested_ = true; 
            cv_.notify_all(); 
        }
    }
}


/******************** RUN ********************/

void EngineController::run()
{
    isRunning_ = true;

    if (!simulationThread_.joinable())
    {
        simulationThread_ = std::thread(&EngineController::simulationLoop, this);    
    }

    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock); 
}


/****************** PAUSE ******************/

void EngineController::pause() 
{
    isRunning_ = false;
}


/************** STEP FORWARD ***************/

void EngineController::stepForward()
{
    if (!isRunning_ && currentTick_ + 1 >= snapshot_.getSize())
    {
        engine_.runTick();
    }

    currentTick_++;
    updateGUI();
}


/**************** STEP BACK ****************/

void EngineController::stepBack()
{
    if (!isRunning_ && currentTick_ > 0)
    {
        --currentTick_;
        updateGUI();
    }
}


/**************** UPDATE GUI ****************/

void EngineController::updateGUI() 
{
    const SimulationState* state = snapshot_.get(currentTick_);
    if (state)
    {
        printGrid(*state, engine_.getGridConfig());
    }
}


/****************** QUIT *******************/

void EngineController::quit()
{
    isRunning_ = false;
    quitRequested_ = true;
}


/*************** IS FINISHED ***************/

bool EngineController::isFinished() const
{
    return engine_.allRobotsReached();
}