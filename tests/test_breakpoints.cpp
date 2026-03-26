#include <gtest/gtest.h>
#include "controller/BreakpointManager.hpp"
#include "controller/TickBreakpoint.hpp"
#include "controller/RobotModeBreakpoint.hpp"
#include "core/SimulationState.hpp"


/**
 * @brief Unit tests for BreakpointManager
 *
 * These tests ensure that breakpoints are correctly added, removed, and managed,
 * and that the correct behavior occurs when checking whether the simulation should break.
 */


// Helper function to create a SimulationState with a robot at a given position
SimulationState makeStateWithRobot(Position pos, int robotId = 0, RobotMode mode = RobotMode::IDLE) 
{
    SimulationState state;
    RobotState robot;
    robot.id = robotId;
    robot.position = pos;
    robot.mode = mode;

    state.robots.push_back(robot);

    return state;
}


// ===============================
// Test for adding Breakpoints
// ===============================

TEST(BreakpointManager, AddTickBreakpoint)
{
    BreakpointManager bpm;
    size_t tick = 5;
    size_t id = bpm.addTickBreakpoint(tick);
    const auto& breakpoints = bpm.getBreakpoints();

    ASSERT_EQ(breakpoints.size(), 1u);
    EXPECT_EQ(breakpoints[0]->getID(), id);
    EXPECT_TRUE(dynamic_cast<TickBreakpoint*>(breakpoints[0].get()) != nullptr);
}


TEST(BreakpointManager, AddRobotModeBreakpoint)
{
    BreakpointManager bpm;
    size_t robotId = 1;
    RobotMode mode = RobotMode::MOVING;
    size_t id = bpm.addRobotBreakpoint(robotId, mode);
    const auto& breakpoints = bpm.getBreakpoints();

    ASSERT_EQ(breakpoints.size(), 1u);
    EXPECT_EQ(breakpoints[0]->getID(), id);
    EXPECT_TRUE(dynamic_cast<RobotModeBreakpoint*>(breakpoints[0].get()) != nullptr);
}


// ===============================
// Test for removing Breakpoints
// ===============================

TEST(BreakpointManager, RemoveBreakpointSuccess)
{
    BreakpointManager bpm;
    size_t tick = 5;
    size_t id = bpm.addTickBreakpoint(tick);
    const auto& breakpoints = bpm.getBreakpoints();
    
    EXPECT_TRUE(bpm.removeBreakpoint(id));
    EXPECT_EQ(breakpoints.size(), 0u);
}


TEST(BreakpointManager, RemoveBreakpointFailure)
{
    BreakpointManager bpm;
    size_t id = 1;

    EXPECT_EQ(bpm.getBreakpoints().size(), 0u);
    EXPECT_FALSE(bpm.removeBreakpoint(id));
}


// ===============================
// Test for `clearAllBreakpoints`
// ===============================

TEST(BreakpointManager, ClearAllBreakpoints)
{
    BreakpointManager bpm;
    bpm.addTickBreakpoint(3);
    bpm.addTickBreakpoint(7);
    bpm.addRobotBreakpoint(1, RobotMode::IDLE);

    EXPECT_EQ(bpm.getBreakpoints().size(), 3u);
    bpm.clearAllBreakpoints();
    EXPECT_EQ(bpm.getBreakpoints().size(), 0u);
}


// ===============================
// Test for shouldBreak method
// ===============================

TEST(BreakpointManager, ShouldBreakTickBreakpoint)
{
    BreakpointManager bpm;
    size_t tick = 7;
    size_t id = bpm.addTickBreakpoint(tick);

    SimulationState state = makeStateWithRobot({0, 0});
    EXPECT_TRUE(bpm.shouldBreak(state, 7));
    EXPECT_FALSE(bpm.shouldBreak(state, 5));
}


TEST(BreakpointManager, ShouldBreakRobotModeBreakpoint)
{
    BreakpointManager bpm;
    size_t robotId = 0;
    RobotMode mode = RobotMode::MOVING;
    size_t id = bpm.addRobotBreakpoint(robotId, mode);

    SimulationState state = makeStateWithRobot({0, 0}, robotId, mode);
    EXPECT_TRUE(bpm.shouldBreak(state, 0));

    state.robots[0].mode = RobotMode::REPLANNING;
    EXPECT_FALSE(bpm.shouldBreak(state, 0));
}


// ============================================
// Test for adding Breakpoints with Unique IDs
// ============================================

TEST(BreakpointManager, AddBreakpointsWithUniqueIDs)
{
    BreakpointManager bpm;
    
    size_t id1 = bpm.addTickBreakpoint(5);
    size_t id2 = bpm.addRobotBreakpoint(1, RobotMode::MOVING);
    size_t id3 = bpm.addTickBreakpoint(10);
    
    const auto& breakpoints = bpm.getBreakpoints();
    
    ASSERT_EQ(breakpoints.size(), 3u);
    
    // Make sure each breakpoint has a unique ID
    ASSERT_NE(id1, id2);
    ASSERT_NE(id1, id3);
    ASSERT_NE(id2, id3);
    EXPECT_EQ(id1, 0u);  
    EXPECT_EQ(id2, 1u);  
    EXPECT_EQ(id3, 2u); 
}



// ================================================
// Test for NextBreakpointID increments correctly
// ================================================

TEST(BreakpointManager, NextBreakpointIDIncrementsCorrectly)
{
    BreakpointManager bpm;
    
    bpm.addTickBreakpoint(3);   
    bpm.addRobotBreakpoint(1, RobotMode::IDLE);  
    bpm.addTickBreakpoint(10);  
    
    EXPECT_EQ(bpm.getBreakpoints().size(), 3u);
    EXPECT_EQ(bpm.getBreakpoints()[0]->getID(), 0u);  
    EXPECT_EQ(bpm.getBreakpoints()[1]->getID(), 1u); 
    EXPECT_EQ(bpm.getBreakpoints()[2]->getID(), 2u);
}


// =============================================
// Test for remove and restore breakpoints ids
// =============================================

TEST(BreakpointManager, RemoveAndRestoreBreakpointID)
{
    BreakpointManager bpm;

    size_t id1 = bpm.addTickBreakpoint(5);   
    size_t id2 = bpm.addRobotBreakpoint(1, RobotMode::PLANNING);  
    size_t id3 = bpm.addTickBreakpoint(10); 

    const auto& breakpoints = bpm.getBreakpoints();
    ASSERT_EQ(breakpoints.size(), 3u);

    
    EXPECT_EQ(breakpoints[0]->getID(), 0u);  
    EXPECT_EQ(breakpoints[1]->getID(), 1u);  
    EXPECT_EQ(breakpoints[2]->getID(), 2u);  

    bool removed = bpm.removeBreakpoint(id2);
    EXPECT_TRUE(removed); 
    ASSERT_EQ(bpm.getBreakpoints().size(), 2u);

    size_t newID = bpm.addTickBreakpoint(15); 
    EXPECT_EQ(newID, 1u);  

    EXPECT_EQ(bpm.getBreakpoints()[0]->getID(), 0u);  
    EXPECT_EQ(bpm.getBreakpoints()[1]->getID(), 2u);  
    EXPECT_EQ(bpm.getBreakpoints()[2]->getID(), 1u); 
}
