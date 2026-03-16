#include <gtest/gtest.h>
#include "planners/BFSPlanner.hpp"
#include "core/SimulationState.hpp"


/**
 * @brief Test suite for BFS Planner on grid-based robots.
 *
 * These tests check:
 * - Basic path finding
 * - Edge cases (start == goal)
 * - No path scenarios (blocked goal)
 * - Optimality (shortest path in steps)
 * - Multiple equal-length paths
 * - Dynamic obstacles
 * - Start/goal outside grid
 */

// ===============================
// Basic functionality tests
// ===============================

TEST(BFSPlanner, FindsPathInEmptyGrid)
{
    BFSPlanner planner;
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


TEST(BFSPlanner, StartEqualsGoal)
{
    BFSPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {2,2};
    robot.goal = {2,2};

    auto path = planner.computePath(state, robot, grid);
    ASSERT_EQ(path.size(), 1);
    EXPECT_EQ(path[0], robot.position);
}


TEST(BFSPlanner, NoPathIfGoalBlocked)
{
    BFSPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {1,0};

    grid.static_obstacles.push_back({1,0}); // block goal
    auto path = planner.computePath(state, robot, grid);
    EXPECT_TRUE(path.empty());
}


TEST(BFSPlanner, FindsShortestPath)
{
    BFSPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {2,2};

    int expectedLength = 4; // number of steps

    auto path = planner.computePath(state, robot, grid);
    EXPECT_EQ(path.size()-1, expectedLength);
}


// ===============================
// Edge cases
// ===============================

TEST(BFSPlanner, ChoosesShortestAmongEquals)
{
    BFSPlanner planner;
    SimulationState state;
    GridConfig grid{3,3};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {2,2};

    // Create multiple shortest paths by blocking middle
    grid.static_obstacles.push_back({1,1});
    auto path = planner.computePath(state, robot, grid);

    int expectedLength = 4;
    EXPECT_EQ(path.size()-1, expectedLength);
    EXPECT_EQ(path.front(), robot.position);
    EXPECT_EQ(path.back(), robot.goal);
}


TEST(BFSPlanner, StartOrGoalOutsideGrid)
{
    BFSPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot1;
    robot1.position = {-1,0}; // start outside
    robot1.goal = {4,4};
    EXPECT_TRUE(planner.computePath(state, robot1, grid).empty());

    RobotState robot2;
    robot2.position = {0,0};
    robot2.goal = {5,5}; // goal outside
    EXPECT_TRUE(planner.computePath(state, robot2, grid).empty());
}


TEST(BFSPlanner, DynamicObstacleChangesPath)
{
    BFSPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {4,0};

    auto path1 = planner.computePath(state, robot, grid);
    EXPECT_EQ(path1.front(), robot.position);
    EXPECT_EQ(path1.back(), robot.goal);

    // Add obstacle blocking direct path
    state.dynamic_obstacles.push_back({2,0});
    auto path2 = planner.computePath(state, robot, grid);

    EXPECT_GT(path2.size(), path1.size());  // new path is longer
    EXPECT_EQ(path2.front(), robot.position);
    EXPECT_EQ(path2.back(), robot.goal);
}