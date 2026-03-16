#include <gtest/gtest.h>
#include "planners/AStarPlanner.hpp"
#include "core/SimulationState.hpp"


/**
 * @brief Test suite for A* Planner on grid-based robots.
 *
 * These tests check:
 * - Basic path finding
 * - Edge cases (start == goal)
 * - No path scenarios
 * - Optimality (Manhattan distance)
 * - Multiple equal-length paths
 * - Start/goal outside the grid
 * - Dynamic obstacles
 * - Heuristic correctness
 */

// ===============================
// Basic functionality tests
// ===============================

TEST(AStarPlanner, FindsPathInEmptyGrid)
{
    AStarPlanner planner;
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


TEST(AStarPlanner, StartEqualsGoal)
{
    AStarPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {2,2};
    robot.goal = {2,2};

    auto path = planner.computePath(state, robot, grid);

    ASSERT_EQ(path.size(), 1);
    EXPECT_EQ(path[0], robot.position);
}


TEST(AStarPlanner, NoPathIfGoalBlocked)
{
    AStarPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {1,0};

    grid.static_obstacles.push_back({1,0});  // block goal

    auto path = planner.computePath(state, robot, grid);

    EXPECT_TRUE(path.empty());
}


TEST(AStarPlanner, FindsOptimalPath)
{
    AStarPlanner planner;
    SimulationState state;
    GridConfig grid{10,10};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {4,3};

    int expectedLength = 7; // Manhattan distance |4| + |3|

    auto path = planner.computePath(state, robot, grid);

    EXPECT_EQ(path.size()-1, expectedLength);
}


// =============================
// Additional edge cases
// =============================

TEST(AStarPlanner, ChoosesOptimalPathAmongEquals)
{
    AStarPlanner planner;
    SimulationState state;
    GridConfig grid{3,3};

    RobotState robot;
    robot.position = {0,0};
    robot.goal = {2,2};

    int expectedLength = 4;

    // Obstacles to create multiple shortest paths of same length
    state.dynamic_obstacles.push_back({1,1}); 
    auto path = planner.computePath(state, robot, grid);

    EXPECT_EQ(path.size()-1, expectedLength);
    EXPECT_EQ(path.front(), robot.position);
    EXPECT_EQ(path.back(), robot.goal);
}


TEST(AStarPlanner, StartOrGoalOutsideGrid)
{
    AStarPlanner planner;
    SimulationState state;
    GridConfig grid{5,5};

    RobotState robot1;
    robot1.position = {-1,0};
    robot1.goal = {4,4};
    auto path1 = planner.computePath(state, robot1, grid);
    EXPECT_TRUE(path1.empty());

    RobotState robot2;
    robot2.position = {0,0};
    robot2.goal = {5,5};
    auto path2 = planner.computePath(state, robot2, grid);
    EXPECT_TRUE(path2.empty());
}


TEST(AStarPlanner, DynamicObstacleChangesPath)
{
    AStarPlanner planner;
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

    EXPECT_GT(path2.size(), path1.size());  // new path longer
    EXPECT_EQ(path2.front(), robot.position);
    EXPECT_EQ(path2.back(), robot.goal);
}