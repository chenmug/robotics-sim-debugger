#pragma once

/**
 * @brief Possible operational states of a robot for state-based breakpoints
 *        and simulation logic.
 *
 * These modes correspond to stages in the robot's sense-plan-act cycle.
 */
enum class RobotMode 
{
    IDLE,        // Robot is idle
    PLANNING,    // Robot is currently planning a path
    MOVING,      // Robot is moving along its planned path
    REPLANNING,  // Robot is replanning its path due to obstacle or update
    GOAL         // Robot reached his goal
};