#include <gtest/gtest.h>
#include "planners/DijkstraPlanner.hpp"
#include "core/SimulationState.hpp"


/**
 * @brief Test suite for Dijkstra Planner on grid-based robots.
 *
 * These tests check:
 * - Basic path finding
 * - Edge cases (start == goal)
 * - No path scenarios (blocked goal)
 * - Optimality (shortest path, uniform-cost)
 * - Dynamic obstacles
 * - Start/goal outside grid
 * - Heuristic function always returns 0
 */

// ===============================
// Basic functionality tests
// ===============================

TEST(DijkstraPlanner, FindsPathInEmptyGrid)
{
    DijkstraPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {4,4};

    auto path = planner.computePath(state, robot, grid);

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), robot.position);
    EXPECT_EQ(path.back(), robot.goal);
}


TEST(DijkstraPlanner, StartEqualsGoal)
{
    DijkstraPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {2,2};
    robot.goal = {2,2};

    auto path = planner.computePath(state, robot, grid);

    ASSERT_EQ(path.size(), 1);
    EXPECT_EQ(path[0], robot.position);
}


TEST(DijkstraPlanner, NoPathIfGoalBlocked)
{
    DijkstraPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {1,0};

    grid.static_obstacles.push_back({1,0}); // block goal

    auto path = planner.computePath(state, robot, grid);
    EXPECT_TRUE(path.empty());
}


TEST(DijkstraPlanner, FindsOptimalPath)
{
    DijkstraPlanner planner;
    SimulationState state;
    GridConfig grid{10,10};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {4,3};

    auto path = planner.computePath(state, robot, grid);
    int expectedLength = 7; // Manhattan distance |4| + |3|

    EXPECT_EQ(path.size()-1, expectedLength);
}


// ===============================
// Additional edge cases
// ===============================

TEST(DijkstraPlanner, ChoosesOptimalPathAmongEquals)
{
    DijkstraPlanner planner;
    SimulationState state;
    GridConfig grid{3,3};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {2,2};

    grid.static_obstacles.push_back({1,1}); // create multiple equal-length paths
    auto path = planner.computePath(state, robot, grid);

    int expectedLength = 4;
    EXPECT_EQ(path.size()-1, expectedLength);
    EXPECT_EQ(path.front(), robot.position);
    EXPECT_EQ(path.back(), robot.goal);
}


TEST(DijkstraPlanner, StartOrGoalOutsideGrid)
{
    DijkstraPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot1;
    robot1.position = {-1,0};
    robot1.goal = {4,4};
    EXPECT_TRUE(planner.computePath(state, robot1, grid).empty());

    RobotState robot2;
    robot2.position = {0,0};
    robot2.goal = {5,5};
    EXPECT_TRUE(planner.computePath(state, robot2, grid).empty());
}


TEST(DijkstraPlanner, DynamicObstacleChangesPath)
{
    DijkstraPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {4,0};

    auto path1 = planner.computePath(state, robot, grid);
    EXPECT_EQ(path1.front(), robot.position);
    EXPECT_EQ(path1.back(), robot.goal);

    state.dynamic_obstacles.push_back({2,0}); // block direct path
    auto path2 = planner.computePath(state, robot, grid);

    EXPECT_GT(path2.size(), path1.size()); // new path is longer
    EXPECT_EQ(path2.front(), robot.position);
    EXPECT_EQ(path2.back(), robot.goal);
}


// ===============================
// Heuristic test
// ===============================

TEST(DijkstraPlanner, HeuristicAlwaysZero)
{
    DijkstraPlanner planner;
    Position a{0,0}, b{4,5};

    EXPECT_EQ(planner.heuristic(a,b), 0);
    EXPECT_EQ(planner.heuristic(b,a), 0);
}