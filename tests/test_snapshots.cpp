#include <gtest/gtest.h>
#include "core/SnapshotManager.hpp"
#include "core/SimulationState.hpp"
#include "robots/GridRobot.hpp"
#include "planners/AStarPlanner.hpp"


/**
 * @brief Unit tests for SnapshotManager
 *
 * These tests ensure that snapshots correctly store and restore simulation states,
 * and that stepping backward behaves correctly.
 */


// Helper to create a simple SimulationState with robot positions
SimulationState makeStateWithRobot(Position pos, int robotId = 0) 
{
    SimulationState state;
    RobotState robot;
    robot.id = robotId;
    robot.position = pos;
    state.robots.push_back(robot);

    return state;
}


// ===============================
// Basic save/get tests
// ===============================

TEST(SnapshotManager, SaveAndGetLast)
{
    SnapshotManager sm;
    auto state1 = makeStateWithRobot({0,0});
    auto state2 = makeStateWithRobot({1,1});

    sm.save(state1);
    sm.save(state2);

    ASSERT_EQ(sm.getSize(), 2u);

    const SimulationState* last = sm.getLast();
    ASSERT_NE(last, nullptr);
    EXPECT_EQ(last->robots[0].position.x, 1);
    EXPECT_EQ(last->robots[0].position.y, 1);
}


TEST(SnapshotManager, GetByTick)
{
    SnapshotManager sm;
    auto state1 = makeStateWithRobot({0,0});
    auto state2 = makeStateWithRobot({2,2});

    sm.save(state1);
    sm.save(state2);

    const SimulationState* s0 = sm.get(0);
    ASSERT_NE(s0, nullptr);
    EXPECT_EQ(s0->robots[0].position.x, 0);

    const SimulationState* s1 = sm.get(1);
    ASSERT_NE(s1, nullptr);
    EXPECT_EQ(s1->robots[0].position.x, 2);

    EXPECT_EQ(sm.get(2), nullptr); // out-of-bounds tick
}


TEST(SnapshotManager, ClearSnapshots)
{
    SnapshotManager sm;
    sm.save(makeStateWithRobot({0,0}));
    sm.save(makeStateWithRobot({1,1}));

    EXPECT_EQ(sm.getSize(), 2u);
    sm.clearSnapshots();
    EXPECT_EQ(sm.getSize(), 0u);
    EXPECT_EQ(sm.getLast(), nullptr);
}


// ===============================
// Step back checks
// ===============================

TEST(SnapshotManager, CanStepBack)
{
    SnapshotManager sm;
    sm.save(makeStateWithRobot({0,0})); // tick 0
    sm.save(makeStateWithRobot({1,1})); // tick 1
    sm.save(makeStateWithRobot({2,2})); // tick 2

    EXPECT_FALSE(sm.canStepBack(0));  // cannot go back from first tick
    EXPECT_TRUE(sm.canStepBack(1));   // can go back from tick 1
    EXPECT_TRUE(sm.canStepBack(2));   // can go back from last tick
    EXPECT_FALSE(sm.canStepBack(3));  // out-of-bounds tick
}


// ===============================
// Snapshot integrity with robots
// ===============================

TEST(SnapshotManager, SnapshotsPreserveRobotPositions)
{
    SnapshotManager sm;

    SimulationState state;
    RobotState r1{0, {0,0}, {9,9}};
    RobotState r2{1, {5,5}, {2,2}};
    state.robots.push_back(r1);
    state.robots.push_back(r2);

    sm.save(state);

    // Move robots and save again
    state.robots[0].position = {1,0};
    state.robots[1].position = {5,6};
    sm.save(state);

    const SimulationState* s0 = sm.get(0);
    const SimulationState* s1 = sm.get(1);

    ASSERT_NE(s0, nullptr);
    ASSERT_NE(s1, nullptr);

    EXPECT_EQ(s0->robots[0].position.x, 0);
    EXPECT_EQ(s0->robots[0].position.y, 0);
    EXPECT_EQ(s0->robots[1].position.x, 5);
    EXPECT_EQ(s0->robots[1].position.y, 5);

    EXPECT_EQ(s1->robots[0].position.x, 1);
    EXPECT_EQ(s1->robots[0].position.y, 0);
    EXPECT_EQ(s1->robots[1].position.x, 5);
    EXPECT_EQ(s1->robots[1].position.y, 6);
}


// =======================================
// Step forward/back simulation scenario
// =======================================

TEST(SnapshotManager, StepForwardAndBackScenario)
{
    SnapshotManager sm;

    // Initial positions
    SimulationState s0 = makeStateWithRobot({0,0}, 0);
    SimulationState s1 = makeStateWithRobot({1,0}, 0);
    SimulationState s2 = makeStateWithRobot({2,0}, 0);

    sm.save(s0);
    sm.save(s1);
    sm.save(s2);

    // Simulate stepping forward and back
    const SimulationState* current = sm.get(0);
    EXPECT_EQ(current->robots[0].position.x, 0);

    current = sm.get(1);
    EXPECT_EQ(current->robots[0].position.x, 1);

    current = sm.get(2);
    EXPECT_EQ(current->robots[0].position.x, 2);

    // Step back from tick 2
    EXPECT_TRUE(sm.canStepBack(2));
    const SimulationState* previous = sm.get(1);
    EXPECT_EQ(previous->robots[0].position.x, 1);
}