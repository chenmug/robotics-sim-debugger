# Robotics Simulation Debugger
### Personal Project — Ongoing Development

**Multi-Threaded Robotics Simulation Framework with Interactive Debugger (C++ / Linux)**

---

## Overview

Robotics Simulation Debugger is a **personal engineering project** developed in modern C++20, designed as a modular simulation framework for modeling robotic systems with clean architecture, concurrency, and interactive state inspection.

The system separates the simulation core from the GUI layer and provides time-control capabilities (run, pause, step forward, step backward) using a snapshot-based state management system.

This project demonstrates:

* Advanced OOP design
* Polymorphism & SOLID principles
* Strategy pattern for plannable robots
* Multi-threaded simulation
* Snapshot-based time travel
* Clean separation between engine and UI
* Linux-based CMake project structure
* Unit testing for core components

---

## Key Features

### 1. Multi-Threaded Simulation

Threads:

* Simulation Thread – advances world state
* GUI Thread – renders state & handles user interaction

Synchronization via:

* `std::mutex`
* `std::condition_variable`
* Thread-safe access to shared state

The GUI never directly mutates simulation state; it reads snapshots from the engine.

---

### 2. Snapshot-Based Time Control (Planned)

The simulation engine is designed to support snapshot-based state management.

In future versions, each simulation tick will store a snapshot containing:

* Robot states
* Environment state

This will enable advanced debugging capabilities such as:

* Pause
* Step forward
* Step backward (rewind)
* Timeline navigation

Snapshots will allow deterministic replay and inspection of previous simulation states.

---

### 3. Modular Robotics Framework

### Robot Abstraction

Robots in the simulation follow a **Sense -> Plan -> Act** execution pipeline.

Each robot observes the current simulation state, plans its next movement using a planner, and then applies the action to the simulation.

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

Concrete implementations:

* `GridRobot` – grid-based robot that navigates using a path planner

---

### Planner Strategy Pattern

Path planning algorithms are implemented using the **Strategy Pattern**.

Each planner computes a path for a robot from its current position to its goal without modifying the simulation state.

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

Implemented Planners:

* `AStarPlanner`
* `DijkstraPlanner`
* `BFSPlanner`

The planner interface allows different algorithms to be swapped at runtime.

The base planner class also provides shared utilities used by search algorithms, including:

* grid boundary checks
* obstacle detection
* position hashing for efficient indexing
* path reconstruction after search completion

---

### 4. Grid-Based Simulation

* Static obstacles
* Dynamic obstacles (added in simulation state)
* Robot positions and goals
* 4-directional movement (up, down, left, right)

---

### 5. GUI (Planned)

A graphical debugger interface is planned using Dear ImGui + SDL2.

The GUI will provide an interactive interface for inspecting and controlling the simulation.

Planned panels include:

**Simulation Control**

* ▶ Run
* ⏸ Pause
* ⏭ Step Forward
* ⏮ Step Back
* Timeline Slider

---

**Grid View**

Visual representation of the environment:

* Obstacles
* Robots
* Goals
* Dynamic updates

---

**Robot Inspector**

Displays:

* Current state
* Active planner
* Position
* Target

---

**Event Log**

Structured output for simulation ticks.

---

## Architecture Overview

```
GUI Layer
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

The GUI interacts only through the Engine Controller.
The Core Engine is headless and testable independently.

---

## Design Patterns Used

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
├── CMakeLists.txt
├── README.md
│
├── include/
│   ├── core/
│   │   ├── SimulationEngine.hpp
│   │   ├── SnapshotManager.hpp
│   │   └── SimulationState.hpp
│   │
│   ├── robots/
│   │   ├── Robot.hpp
│   │   └── GridRobot.hpp
│   │
│   ├── planners/
│   │   ├── Planner.hpp
│   │   ├── GraphSearchPlanner.hpp
│   │   ├── AStarPlanner.hpp
│   │   ├── DijkstraPlanner.hpp
│   │   └── BFSPlanner.hpp
│   │
│   └── controller/
│       └── EngineController.hpp
│
├── src/
│   ├── core/
│   ├── robots/
│   ├── planners/
│   ├── controller/
│   └── main.cpp
│
├── gui/
│   ├── GuiApp.cpp
│   ├── GridRenderer.cpp
│   ├── InspectorPanel.cpp
│   └── LogPanel.cpp
│
├── tests/
│   ├── test_planners.cpp
│   ├── test_snapshot.cpp
│   └── CMakeLists.txt
│
└── assets/
    └── demo_scenarios/
```

---

## Build Instructions (Linux)

Requirements:

* C++20 compatible compiler
* CMake ≥ 3.16

Planned dependencies:

* SDL2
* Dear ImGui
* GoogleTest

Build:

```
mkdir build
cd build
cmake ..
make
```

Run:

```
./robotics_sim_debugger
```

Run tests:

```
ctest
```

---

## Testing Strategy

Testing infrastructure using **GoogleTest** is planned for future development.

Planned tests include:

* Unit tests for path planning algorithms
* Snapshot consistency verification
* Deterministic simulation behavior tests
* Mocked components for isolated testing

---

## Future Work / Ongoing Development

* **Collision Avoidance Between Robots** – dynamic replanning to prevent collisions in multi-robot scenarios
* **GUI / Visualization** – interactive view with Dear ImGui or other graphics
* **Sensor Integration** – lidar/proximity sensors per robot
* **Dynamic Obstacles** – moving obstacles that require replanning
* **Threading / Real-Time Simulation** – multi-threaded execution with safe state access
* **Replay & Snapshot Navigation** – step forward/back and timeline control
* **Logging & Event Handling** – structured simulation events and debug logs

---

## Motivation

This project is a **systems-oriented robotics simulation tool** emphasizing architecture, extensibility, and debuggability rather than graphics-heavy visualization.

It reflects a focus on:

- Core software engineering  
- Modern C++ practices  
- Robust architecture  
- Practical systems design

---

#### Current state is fully functional for grid-based path planning and console simulation, but several advanced features are in progress.