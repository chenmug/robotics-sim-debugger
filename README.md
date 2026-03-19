# Robotics Simulation Debugger
### Personal Project — Ongoing Development

**Multi-Threaded Robotics Simulation Framework with Interactive Debugger (C++ / Linux)**

---

## Overview

Robotics Simulation Debugger is a **personal engineering project** developed in modern C++20. It provides a modular simulation framework for robotic systems with clean architecture, concurrency, and interactive state inspection.

The system separates the simulation core from the UI and provides **time-control capabilities** (run, pause, step forward/backward) using a **snapshot-based state management system**.

Additionally, the project is evolving into a **deterministic debugging framework**, enabling deep inspection of system behavior through replay and breakpoint mechanisms.

---

## Key Features

### 1. Multi-Threaded Simulation

* Simulation Thread – advances world state
* GUI Thread (planned) – reads snapshots & handles user interaction
* Synchronization with `std::mutex` and `std::condition_variable`
* GUI/Console never directly mutates simulation state; it reads snapshots

---

### 2. Snapshot-Based Time Control & Replay

* Each simulation tick automatically saves a snapshot
* Snapshots store robot states and environment state
* Supports:

  * **Step forward / backward**
  * **Replay from the beginning (planned)**
  * **Jump to specific tick (planned)**
  * **Play forward from arbitrary state (planned)**

Snapshots ensure **deterministic execution**, allowing reproducible debugging and consistent system analysis.

---

### 3. Breakpoint System (Planned)

The system is being extended with a **debugger-style breakpoint mechanism**, allowing automatic pause of the simulation when specific conditions are met.

Planned breakpoint types:

#### Tick-Based Breakpoints

Pause execution at a specific simulation step:

```
Break at tick = 100
```

#### State-Based Breakpoints

Pause when a robot enters a specific state:

```
Break when RobotState == REPLANNING
```

#### Event-Based Breakpoints (Future Extension)

Pause when specific events occur:

```
Break on ObstacleDetected
Break on ReplanTriggered
```

This transforms the simulator into a **domain-specific debugging tool** for robotic systems.

---

### 4. Modular Robotics Framework

#### Robot Abstraction

* Sense -> Plan -> Act pipeline
* `GridRobot` as a concrete implementation

```cpp
class Robot
{
protected:
    size_t id_ = 0;  // unique robot identifier

public:
    virtual void sense(const SimulationState& state) = 0;
    virtual void plan(const SimulationState& state) = 0;
    virtual void act(SimulationState& state) = 0;

    virtual ~Robot() = default;
};
```

**Note:** This is a **partial interface**. Additional functions and internal members exist in the full implementation.

---

#### Planner Strategy Pattern

* Path planners implement **Strategy Pattern**
* Planners compute paths without modifying simulation state
* Implemented: `AStarPlanner`, `DijkstraPlanner`, `BFSPlanner`

```cpp
class Planner 
{
public:
    virtual std::vector<Position> computePath(
        const SimulationState& state,
        const RobotState& robot,
        const GridConfig& grid) = 0;

    virtual ~Planner() = default;
};
```

---

### 5. Grid-Based Simulation

* Static and dynamic obstacles
* Robot positions and goals
* 4-directional movement (up/down/left/right)
* Real-time console rendering

---

### 6. Console-Based Debugger

Commands:

* `n` — step forward
* `b` — step backward
* `r` — run continuously
* `p` — pause
* `q` — quit

Snapshots ensure reliable forward/back stepping; each tick corresponds to a saved state.

Planned extensions:

* Breakpoint-triggered pause
* Timeline navigation
* Replay controls

---

### 7. Performance Benchmarking (Planned)

The system will include built-in instrumentation for measuring simulation performance.

Planned metrics:

* Tick execution time (ms)
* Planner computation time
* Number of processed actions per tick
* Pathfinding complexity (e.g., nodes explored)

These metrics will be used for both debugging and performance analysis.

---

### 8. GUI (Planned)

* Dear ImGui + SDL2
* Simulation control panel
* Grid visualization
* Robot inspector
* Event log
* Timeline slider
* Breakpoint management UI

---

## Architecture Overview

```
GUI Layer (planned)
   ↓
Engine Controller
   ↓
Core Simulation Engine
   ├── Robots
   ├── Planners
   ├── Sensors
   ├── Snapshot Manager
   └── Simulation Loop
```

* GUI interacts only through Engine Controller
* Core Engine is headless and testable independently
* SnapshotManager ensures deterministic replay

---

## Design Patterns

* **Strategy Pattern** – Path planners
* **State Machine** – Robot execution cycle
* **Factory Pattern** – Robot / planner creation
* **RAII** – Safe resource management

---

## Memory Management

* `std::unique_ptr` for ownership
* `std::shared_ptr` where shared lifetime is required
* No raw owning pointers
* Thread-safe state access via controlled interfaces

---

## Project Structure

```
robotics-sim-debugger/
│
├── include/
│   ├── core/
│   ├── robots/
│   ├── planners/
│   └── controller/
│
├── src/
│   ├── core/
│   ├── robots/
│   ├── planners/
│   └── controller/
├── gui/ (planned)
├── tests/
└── assets/
```

---

## Build Instructions (Linux)

Requirements:

* C++20 compatible compiler
* CMake ≥ 3.16

Build:

```bash
mkdir build
cd build
cmake ..
make
```

Run:

```bash
./robotics_sim_debugger
```

Run tests:

```bash
ctest
```

---

## Testing & Benchmark Strategy

* GoogleTest for unit testing
* Snapshot consistency tests
* Step forward/back behavior
* Deterministic simulation behavior
* Planned:

  * Benchmarking per tick
  * Breakpoint behavior validation

---

## Future Work

* GUI / Visualization
* Dynamic Obstacles
* Multi-Threaded Real-Time Simulation
* Timeline Navigation (jump to tick)
* Breakpoint System
* Performance Benchmarking

---

## Motivation

A **systems-oriented robotics simulation tool** emphasizing architecture, extensibility, and debuggability rather than graphics-heavy visualization.

Focus:

* Core software engineering
* Object-Oriented Programming (OOP) principles
* Modern C++ practices
* Robust architecture
* Practical systems design

---

#### Current State

* Fully functional **grid-based path planning**
* Console simulation with working **snapshots** and **step forward/back**
* Deterministic execution enabled via snapshot system
* GUI and advanced debugging features are in progress
* Pause (`p`) works correctly even while running (`r`)