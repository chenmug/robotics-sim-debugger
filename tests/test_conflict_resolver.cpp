#include <gtest/gtest.h>
#include "core/SimulationState.hpp"
#include "core/ConflictResolver.hpp"
#include "core/GridConfig.hpp"


// Helper to create a SimulationState with GridRobots
SimulationState createSimulationStateWithRobots(
    const std::vector<std::pair<Position, Position>>& robotsPos)
{
    SimulationState state;
    size_t n = robotsPos.size();

    for (size_t i = 0; i < n; ++i)
    {
        const auto& [start, goal] = robotsPos[i];

        RobotState robot;
        robot.id = i;
        robot.position = start;
        robot.goal = goal;
        robot.nextPlannedPos = start;
        robot.wasBlocked = false;
        robot.blockedNow = false;
        robot.planned_path.clear();
        robot.path_index = 0;
        robot.mode = RobotMode::IDLE;

        state.robots.push_back(robot);
    }

    state.tick = 0;
    return state;
}


// ========================
// ConflictResolver Tests
// ========================

// First-time swap conflict -> both wait
TEST(ConflictResolver, FirstTimeSwapConflictBothWait)
{
    GridConfig grid{5, 5};
    SimulationState state = createSimulationStateWithRobots({
        {{0,0},{1,0}}, {{1,0},{0,0}}});

    state.robots[0].nextPlannedPos = {1,0};
    state.robots[1].nextPlannedPos = {0,0};

    ConflictResolver resolver;
    resolver.resolve(state, grid);

    EXPECT_TRUE(state.robots[0].blockedNow);
    EXPECT_TRUE(state.robots[1].blockedNow);

    EXPECT_EQ(state.robots[0].nextPlannedPos, state.robots[0].position);
    EXPECT_EQ(state.robots[1].nextPlannedPos, state.robots[1].position);
}


// Repeated conflict -> exactly one robot proceeds, one moves aside
TEST(ConflictResolver, RepeatedConflictOneMovesAside)
{
    GridConfig grid{5, 5};
    SimulationState state = createSimulationStateWithRobots({
        {{0,0},{1,0}}, {{1,0},{0,0}}});

    ConflictResolver resolver;

    // First tick -> both wait
    state.robots[0].nextPlannedPos = {1,0};
    state.robots[1].nextPlannedPos = {0,0};
    resolver.resolve(state, grid);

    // simulate carry-over
    for (auto& r : state.robots)
    {
        r.wasBlocked = r.blockedNow;
    }

    // Second tick -> same conflict
    state.robots[0].nextPlannedPos = state.robots[0].goal;
    state.robots[1].nextPlannedPos = state.robots[1].goal;

    resolver.resolve(state, grid);

    int movedToGoal = 0;
    int movedAside = 0;

    for (const auto& r : state.robots)
    {
        if (r.nextPlannedPos == r.goal)
        {
            movedToGoal++;
        }
        if (r.blockedNow)
        {
            movedAside++;
        }
    }

    EXPECT_EQ(movedToGoal, 1);
    EXPECT_EQ(movedAside, 1);
}


// No conflict
TEST(ConflictResolver, NoConflictWhenPathsDoNotCollide)
{
    GridConfig grid{5,5};
    SimulationState state = createSimulationStateWithRobots({
        {{0,0},{1,0}}, {{2,0},{3,0}}});

    state.robots[0].nextPlannedPos = {1,0};
    state.robots[1].nextPlannedPos = {3,0};

    ConflictResolver resolver;
    resolver.resolve(state, grid);

    EXPECT_FALSE(state.robots[0].blockedNow);
    EXPECT_FALSE(state.robots[1].blockedNow);

    EXPECT_EQ(state.robots[0].nextPlannedPos, Position({1,0}));
    EXPECT_EQ(state.robots[1].nextPlannedPos, Position({3,0}));
}


// Robot already at goal should not interfere
TEST(ConflictResolver, RobotAtGoalIgnored)
{
    GridConfig grid{5,5};
    SimulationState state = createSimulationStateWithRobots({
        {{0,0},{0,0}},  // already at goal
        {{1,0},{2,0}}
    });

    state.robots[0].nextPlannedPos = {0,0};
    state.robots[1].nextPlannedPos = {2,0};

    ConflictResolver resolver;
    resolver.resolve(state, grid);

    EXPECT_FALSE(state.robots[0].blockedNow);
    EXPECT_EQ(state.robots[0].nextPlannedPos, state.robots[0].position);

    EXPECT_FALSE(state.robots[1].blockedNow);
    EXPECT_EQ(state.robots[1].nextPlannedPos, Position({2,0}));
}


// Same cell conflict
TEST(ConflictResolver, SameCellConflictBothWait)
{
    GridConfig grid{5,5};
    SimulationState state = createSimulationStateWithRobots({
        {{0,0},{2,0}}, {{1,0},{2,0}}});

    state.robots[0].nextPlannedPos = {1,0};
    state.robots[1].nextPlannedPos = {1,0};

    ConflictResolver resolver;
    resolver.resolve(state, grid);

    EXPECT_TRUE(state.robots[0].blockedNow);
    EXPECT_TRUE(state.robots[1].blockedNow);

    EXPECT_EQ(state.robots[0].nextPlannedPos, state.robots[0].position);
    EXPECT_EQ(state.robots[1].nextPlannedPos, state.robots[1].position);
}


// Stuck conflict (both were blocked and not moving)
TEST(ConflictResolver, StuckConflictHandled)
{
    GridConfig grid{5,5};
    SimulationState state = createSimulationStateWithRobots({
        {{0,0},{1,0}},{{1,0},{0,0}}});

    // simulate stuck state
    for (auto& r : state.robots)
    {
        r.wasBlocked = true;
        r.nextPlannedPos = r.position; // not moving
    }

    ConflictResolver resolver;
    resolver.resolve(state, grid);

    // One should try to move aside (depending on priority)
    int blockedCount = 0;
    for (const auto& r : state.robots)
    {
        if (r.blockedNow)
        {
            blockedCount++;
        }
    }

    EXPECT_GE(blockedCount, 1);
}


// No free neighbor → robot stays in place when trying to move aside
TEST(ConflictResolver, NoFreeNeighborRobotStays)
{
    GridConfig grid{2,2};

    SimulationState state = createSimulationStateWithRobots({
        {{0,0},{1,1}},
        {{0,1},{1,0}},
        {{1,0},{0,1}},
        {{1,1},{0,0}}
    });

    ConflictResolver resolver;

    // First tick: all wait
    state.robots[0].nextPlannedPos = {0,1};
    state.robots[1].nextPlannedPos = {0,0};
    resolver.resolve(state, grid);

    for (auto& r : state.robots)
        r.wasBlocked = r.blockedNow;

    // Second tick: conflict again
    state.robots[0].nextPlannedPos = {0,1};
    state.robots[1].nextPlannedPos = {0,0};

    resolver.resolve(state, grid);

    EXPECT_EQ(state.robots[1].nextPlannedPos, state.robots[1].position);
    EXPECT_TRUE(state.robots[1].blockedNow);
}