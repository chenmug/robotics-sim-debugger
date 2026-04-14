#include "ui/console/ConsoleRenderer.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <map>


// /************* CONST VARIABLES ***************/

const int TOTAL_TICKS = 18;
const int INNER_WIDTH = 70;
const int CELL_WIDTH = 7;
const int BAR_WIDTH = 20;
const int COL_WIDTH = 28;
const int PERF_WIDTH = 38;
const int INSPECT_WIDTH = 28;


// /********* STATIC HELPER FUNCTIONS ***********/

// /*********** IS IN PATH **************/

static bool isInPath(const std::vector<Position>& path, const Position& pos)
{
    for (const auto& p : path)
    {
        if (p == pos)
        {
            return true;
        }
    }

    return false;
}


// /*********** IS GOAL CELL ************/

static bool isGoalCell(const SimulationState& state, const Position& pos)
{
    for (size_t i = 0; i < state.robots.size(); ++i)
    {
        if (state.robots[i].goal == pos)
            return true;
    }

    return false;
}


// /********* STRINGS HELPER FUNCTIONS **********/

// /******* EVENT TYPE TO STRING **********/

std::string eventTypeToString(EventType type)
{
    switch(type)
    {
        case EventType::OBSTACLE_DETECTED: 
            return "ObstacleDetected";

        case EventType::REPLAN_TRIGGERED:  
            return "ReplanTriggered";

        case EventType::AVOID_COLLISION: 
            return "AvoidCollision";

        case EventType::GOAL_REACHED:
            return "GoalReached";

        default: return "UnknownEvent";
    }
}


// /******* ROBOT MODE TO STRING **********/

std::string robotModeToString(RobotMode mode)
{
    switch (mode)
    {
        case RobotMode::IDLE: 
            return "IDLE";

        case RobotMode::PLANNING: 
            return "PLANNING";

        case RobotMode::MOVING: 
            return "MOVING";

        case RobotMode::REPLANNING:
            return "REPLANNING";

        case RobotMode::GOAL: 
            return "GOAL";

        default: 
            return "UNKNOWN";
    }
}




// /*************** RENDER VIEW *****************/

void renderView(const DebugSnapshotView& view)
{
    std::system("clear");

    renderHeader(view);
    renderGrid(view);
    renderRobotsEventsSideBySide(view);  
    renderPerfWithInspector(view);
    renderTimeline(view);
    renderFooter();
}


// /************** RENDER HEADER ****************/

void renderHeader(const DebugSnapshotView& view)
{
    std::string mode = (view.isRunning ? "RUNNING" : "PAUSED");

    int totalTicks = TOTAL_TICKS;
    int robotCount = view.state ? view.state->robots.size() : 0;

    std::stringstream line;

    line << "  Tick: " << view.tick << " / " << totalTicks
         << "  |  Mode: " << std::left << std::setw(7) << mode
         << "  |  Robots: " << robotCount
         << "  |  Snapshot: # " << view.tick;

    std::string content = line.str();

    if ((int)content.length() < INNER_WIDTH)
        content += std::string(INNER_WIDTH - content.length() - 1, ' ');

    std::stringstream header;

    header << "╔══════════════════════════════════════════════════════════════════════╗\n";
    header << "║                     Robotics Simulation Debugger                     ║\n";
    header << "║ " << content << "║\n";
    header << "╚══════════════════════════════════════════════════════════════════════╝\n";

    std::cout << header.str();
}


// /*************** RENDER GRID *****************/

void renderGrid(const DebugSnapshotView& view)
{
    if (!view.state || !view.grid)
    {
        std::cout << "GRID VIEW: (no data)\n\n";
        return;
    }

    const auto& state = *view.state;
    const auto& grid  = *view.grid;
    int selectedId = view.selectedRobotId;

    bool hasPath = (selectedId >= 0 && selectedId < (int)view.robotsInfo.size() &&
                    !view.robotsInfo[selectedId].path.empty());

    const std::vector<Position>& path =
        hasPath ? view.robotsInfo[selectedId].path : std::vector<Position>{};

    std::cout << "GRID VIEW:\n";
    std::cout << "┌";

    for (int x = 0; x < grid.width * CELL_WIDTH; ++x)
    {
        std::cout << "─";
    }
    std::cout << "┐\n";

    for (int y = 0; y < grid.height; ++y)
    {
        std::cout << "│";

        for (int x = 0; x < grid.width; ++x)
        {
            Position pos{x, y};
            std::string cell = ".";
            bool isRobot = false;

            // robots (highest priority)
            for (size_t i = 0; i < state.robots.size(); ++i)
            {
                if (state.robots[i].position == pos)
                {
                    if ((int)i == selectedId)
                    {
                        cell = "[R" + std::to_string(i + 1) + "]";
                    }
                    else
                    {
                        cell = "R" + std::to_string(i + 1);
                    }

                    isRobot = true;
                    break;
                }
            }

            // Mark the path of the selected robot
            if (!isRobot && hasPath && isInPath(path, pos))
            {
                if (!isGoalCell(state, pos))
                {
                    cell = "*";
                }
            }
            // Obstacles
            if (cell == ".")
            {
                for (const auto& o : grid.static_obstacles)
                {
                    if (o == pos) cell = "#";
                }

                for (const auto& o : state.dynamic_obstacles)
                {
                    if (o == pos) cell = "#";
                }
            }
            // Goals
            if (cell == ".")
            {
                for (size_t i = 0; i < state.robots.size(); ++i)
                {
                    if (state.robots[i].goal == pos)
                    {
                        cell = "G" + std::to_string(i + 1);
                        break;
                    }
                }
            }

            std::cout << " " << std::left << std::setw(CELL_WIDTH - 1) << cell;
        }

        std::cout << "│\n";
    }

    std::cout << "└";

    for (int x = 0; x < grid.width * CELL_WIDTH; ++x)
    {
        std::cout << "─";
    }

    std::cout << "┘\n";
    std::cout << " R = Robot   [R] = Selected   # = Obstacle   G = Goal   * = Path\n\n";
}


// /******** BUILD ROBOT LINES - HELPER *********/

static std::vector<std::string> buildRobotLines(const DebugSnapshotView& view)
{
    std::vector<std::string> lines;

    if (!view.state)
    { 
        return lines;
    }

    const auto& state = *view.state;

    for (size_t i = 0; i < state.robots.size(); ++i)
    {
        const auto& r = state.robots[i];
        std::stringstream out;

        if ((int)i == view.selectedRobotId)
        {
            out << "> ";   
        }
        else
        {
            out << "  ";
        }

        out << "R" << i + 1
           << " (" << r.position.x << "," << r.position.y << ")"
           << "->(" << r.goal.x << "," << r.goal.y << ") "
           << robotModeToString(r.mode);

        lines.push_back(out.str());
    }

    return lines;
}


// /******** BUILD EVENT LINES - HELPER *********/

static std::vector<std::string> buildEventLines(const DebugSnapshotView& view)
{
    std::vector<std::string> lines;

    if (!view.state || view.state->events.empty())
    {
        lines.push_back("(none)");
        return lines;
    }

    for (const auto& e : view.state->events)
    {
        std::stringstream out;
        out << "R" << e.robotId + 1 << " -> " << eventTypeToString(e.type);
        lines.push_back(out.str());
    }

    return lines;
}


// /********** RENDER ROBOT & EVENTS ************/

void renderRobotsEventsSideBySide(const DebugSnapshotView& view)
{
    auto left  = buildRobotLines(view);
    auto right = buildEventLines(view);

    size_t maxLines = std::max(left.size(), right.size());
    const std::string gap(8, ' ');

    std::cout << "┌─────────── ROBOTS ───────────┐"
              << gap
              << "┌─────────── EVENTS ───────────┐\n";

    for (size_t i = 0; i < maxLines; ++i)
    {
        std::string l = (i < left.size()) ? left[i] : "";
        std::string r = (i < right.size()) ? right[i] : "";

        std::cout << "│ " << std::left << std::setw(COL_WIDTH) << l
                  << " │" << gap << "│ " << std::left << std::setw(COL_WIDTH) << r
                  << " │\n";
    }

    std::cout << "└──────────────────────────────┘"
              << gap
              << "└──────────────────────────────┘\n\n";
}


// /******** BUILD PREF LINES - HELPER **********/

static std::vector<std::string> buildPerfLines(const DebugSnapshotView& view)
{
    std::vector<std::string> lines;

    if (view.robotsInfo.empty())
    {
        lines.push_back("(no perf data)");
        return lines;
    }

    for (const auto& r : view.robotsInfo)
    {
        std::ostringstream out;
        out << std::left
            << std::setw(4) << r.name << " "
            << std::setw(10) << r.algorithm
            << std::setw(10) << (std::to_string(r.nodesExpanded) + " nodes")
            << std::fixed << std::setprecision(3)
            << r.plannerTimeMs << " ms";

        lines.push_back(out.str());
    }

    return lines;
}


// /****** BUILD INSPECTOR LINES - HELPER *******/

static std::vector<std::string> buildInspectorLines(const DebugSnapshotView& view)
{
    std::vector<std::string> lines;

    if (view.selectedRobotId < 0 || !view.state)
    {
        lines.push_back("│   No robot selected   │");
        lines.push_back("└───────────────────────┘");
        return lines;
    }

    int id = view.selectedRobotId;
    const auto& state = *view.state;
    const auto& r = state.robots[id];

    auto makeLine = [](const std::string& label, const std::string& value)
    {
        std::stringstream ss;
        ss << "│ " << std::left << std::setw(12) << label
           << std::setw(10) << value << "│";
        return ss.str();
    };

    lines.push_back(makeLine("Focus:", "R" + std::to_string(id + 1)));
    lines.push_back(makeLine("State:", robotModeToString(r.mode)));

    if (id < (int)view.robotsInfo.size())
    {
        const auto& info = view.robotsInfo[id];
        lines.push_back(makeLine("Planner:", info.algorithm));
        lines.push_back(makeLine("Path len:", std::to_string(info.pathLength)));
        lines.push_back(makeLine("Nodes:", std::to_string(info.nodesExpanded)));
    }

    int eventCount = 0;
    for (const auto& e : state.events)
    {
        if (e.robotId == (size_t)id)
        {
            eventCount++;
        }
    }

    lines.push_back(makeLine("events:", std::to_string(eventCount)));
    lines.push_back("└───────────────────────┘");

    return lines;
}


// /******** RENDER PREF WITH INSPECTOR *********/

void renderPerfWithInspector(const DebugSnapshotView& view)
{
    auto perfLines = buildPerfLines(view);
    auto inspectorLines = buildInspectorLines(view);

    size_t maxLines = std::max(perfLines.size(), inspectorLines.size());
    const std::string gap(6, ' ');

    // headers
    std::cout << "┌───────────── PERFORMANCE ─────────────┐"
              << gap
              << "┌────── INSPECTOR ──────┐\n";

    for (size_t i = 0; i < maxLines; ++i)
    {
        // Pref
        std::string p = (i < perfLines.size()) ? perfLines[i] : "";

        std::cout << "│ "
                << std::left << std::setw(PERF_WIDTH) << p
                << "│";

        // Inspector
        std::string ins = (i < inspectorLines.size()) ? inspectorLines[i] : "";

        if (!ins.empty())
            std::cout << gap << ins;
        else
            std::cout << gap << " "; 

        std::cout << "\n";
    }

    std::cout << "└───────────────────────────────────────┘\n";
}


// /************* RENDER TIMELINE ***************/

void renderTimeline(const DebugSnapshotView& view)
{
    int current = view.tick;
    int total = TOTAL_TICKS;
    int filled = (current * BAR_WIDTH) / total;
    int percent = (total > 0) ? (current * 100) / total : 0;

    std::string bar;
    for (int i = 0; i < BAR_WIDTH; ++i)
    {
        bar += (i < filled) ? "#" : "-";
    }

    std::cout << "Timeline: [" << bar << "] " << current << " / " << total 
              << " (" << percent << "%)\n\n";
}


// /************** RENDER FOOTER ****************/

void renderFooter()
{
    std::cout << "╔══════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║    [n]ext | [b]ack | [r]un | [p]ause | [j]ump | [s]elect | [q]uit    ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝\n";
}