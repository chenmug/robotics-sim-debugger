#include <gtest/gtest.h>
#include "core/SimulationEngine.hpp"
#include "core/SnapshotManager.hpp"
#include "robots/GridRobot.hpp"
#include "planners/AStarPlanner.hpp"
#include "controller/EngineController.hpp"


// Helper to create a SimulationEngine with one GridRobot
SimulationEngine makeEngineWithRobot(Position start, Position goal, GridConfig grid = {5,5})
{
    SimulationEngine engine(grid);
    auto robot = std::make_unique<GridRobot>(grid, std::make_shared<AStarPlanner>());
    engine.addRobot(std::move(robot), start, goal);

    return engine;
}


// ===============================
// EngineController Tests
// ===============================

TEST(EngineController, StepForwardAndBack)
{
    GridConfig grid{5,5};
    SimulationEngine engine(grid);

    auto robot = std::make_unique<GridRobot>(grid, std::make_shared<AStarPlanner>());
    engine.addRobot(std::move(robot), {0,0}, {2,2});

    SnapshotManager& snapshots = engine.getSnapshotManager();
    EngineController controller{engine, snapshots};

    // Save initial state
    snapshots.save(engine.getCurrentState());

    Position startPos = engine.getCurrentState().robots[0].position;
    Position goalPos  = engine.getCurrentState().robots[0].goal;

    // Step forward 1
    controller.stepForward();
    Position posAfterStep1 = engine.getCurrentState().robots[0].position;

    // Step forward 2
    controller.stepForward();
    Position posAfterStep2 = engine.getCurrentState().robots[0].position;

    // Step back to tick 1
    controller.stepBack();
    Position posAfterBack1 = engine.getCurrentState().robots[0].position;

    // Step back to tick 0
    controller.stepBack();
    Position posAfterBack2 = engine.getCurrentState().robots[0].position;

    // Step forward to tick 1 again
    controller.stepForward();
    Position posAfterStep3 = engine.getCurrentState().robots[0].position;

    // Step forward to tick 2
    controller.stepForward();
    Position posAfterStep4 = engine.getCurrentState().robots[0].position;

    // --- Assertions ---

    // Snapshots exist
    const SimulationState* tick0 = snapshots.get(0);
    const SimulationState* tick1 = snapshots.get(1);
    const SimulationState* tick2 = snapshots.get(2);

    ASSERT_NE(tick0, nullptr);
    ASSERT_NE(tick1, nullptr);
    ASSERT_NE(tick2, nullptr);

    // Positions per tick
    EXPECT_EQ(tick0->robots[0].position, startPos);
    EXPECT_EQ(tick1->robots[0].position, posAfterStep1);
    EXPECT_EQ(tick2->robots[0].position, posAfterStep2);

    // Goal stays the same
    EXPECT_EQ(tick0->robots[0].goal, goalPos);
    EXPECT_EQ(tick1->robots[0].goal, goalPos);
    EXPECT_EQ(tick2->robots[0].goal, goalPos);

    // StepBack/Forward works correctly
    EXPECT_EQ(posAfterBack1, posAfterStep1);
    EXPECT_EQ(posAfterBack2, startPos);
    EXPECT_EQ(posAfterStep3, posAfterStep1);
    EXPECT_EQ(posAfterStep4, posAfterStep2);

    // Engine should not be finished yet
    EXPECT_FALSE(controller.isFinished());

    // Simulate final tick to reach goal
    while (!controller.isFinished())
    {
        controller.stepForward();
    }

    const SimulationState* finalTick = snapshots.get(controller.getCurrentTick());
    ASSERT_NE(finalTick, nullptr);
    EXPECT_EQ(finalTick->robots[0].position, goalPos);
    EXPECT_TRUE(controller.isFinished());
}


TEST(EngineController, IsFinishedWithMultipleRobots)
{
    GridConfig grid{5,5};
    SimulationEngine engine(grid);

    auto r1 = std::make_unique<GridRobot>(grid, std::make_shared<AStarPlanner>());
    auto r2 = std::make_unique<GridRobot>(grid, std::make_shared<AStarPlanner>());

    engine.addRobot(std::move(r1), {0,0}, {1,0});
    engine.addRobot(std::move(r2), {0,1}, {1,1});

    SnapshotManager& snapshots = engine.getSnapshotManager();
    EngineController controller{engine, snapshots};

    snapshots.save(engine.getCurrentState());

    while (!engine.allRobotsReached())
        controller.stepForward();

    EXPECT_TRUE(controller.isFinished());
}


TEST(EngineController, StepBackDoesNotGoNegative)
{
    GridConfig grid{5,5};
    SimulationEngine engine(grid);
    auto robot = std::make_unique<GridRobot>(grid, std::make_shared<AStarPlanner>());
    engine.addRobot(std::move(robot), {0,0}, {2,0});

    Position p{0,0};

    SnapshotManager& snapshots = engine.getSnapshotManager();
    EngineController controller{engine, snapshots};

    snapshots.save(engine.getCurrentState());

    // Step back on first tick - should remain at 0
    controller.stepBack();
    EXPECT_EQ(engine.getCurrentState().robots[0].position, p);
}


TEST(EngineController, SnapshotsAreUsedOnStepForward)
{
    GridConfig grid{5,5};
    SimulationEngine engine(grid);
    auto robot = std::make_unique<GridRobot>(grid, std::make_shared<AStarPlanner>());
    engine.addRobot(std::move(robot), {0,0}, {2,0});

    SnapshotManager& snapshots = engine.getSnapshotManager();
    EngineController controller{engine, snapshots};

    snapshots.save(engine.getCurrentState());

    // Step forward - snapshot exists
    controller.stepForward();
    const SimulationState* snap1 = snapshots.get(1);
    ASSERT_NE(snap1, nullptr);
    EXPECT_EQ(engine.getCurrentState().robots[0].position, snap1->robots[0].position);
}


TEST(EngineController, StepForwardCreatesSnapshotIfMissing)
{
    GridConfig grid{5,5};
    SimulationEngine engine(grid);
    auto robot = std::make_unique<GridRobot>(grid, std::make_shared<AStarPlanner>());
    engine.addRobot(std::move(robot), {0,0}, {2,0});

    SnapshotManager& snapshots = engine.getSnapshotManager();
    EngineController controller{engine, snapshots};

    controller.stepForward(); 

    EXPECT_EQ(snapshots.getSize(), 1u); 
    EXPECT_EQ(engine.getCurrentState().robots[0].position, snapshots.getLast()->robots[0].position);
}