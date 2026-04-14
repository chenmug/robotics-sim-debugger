# Robotics Simulation Debugger
### Personal Project — Ongoing Development

### Deterministic Multi-Agent Simulation Engine with Time-Travel Debugging (C++ / Linux)

---

## Overview

Robotics Simulation Debugger is a **personal systems-oriented C++20 project** focused on building a deterministic simulation engine with advanced debugging capabilities.

The project enables **deep inspection and reproducibility** of complex robotic systems, particularly in the presence of **concurrency and multi-agent interactions**.

Unlike traditional simulators, this system is designed as a **debuggable engine**, not just a visual tool.

---

## Why This Project Exists

Debugging robotics simulations is inherently difficult due to:

- Non-deterministic behavior  
- Concurrent execution  
- Complex interactions between multiple agents  

This project explores how to transform a simulation into a **deterministic, inspectable system**, enabling:

- Reproducible execution  
- Time-travel debugging (forward/backward stepping)  
- Controlled analysis of system behavior  

---

## Key Capabilities

- Deterministic execution  
- Snapshot-based state management  
- Time-travel debugging (step forward/backward)  
- Multi-threaded simulation architecture  
- Multi-agent coordination with conflict resolution  
- Modular robotics framework  
- Performance Benchmarking: Ability to measure and print various performance metrics, including planner execution time and nodes expanded.

---

## Core Features

### Deterministic Simulation Engine

- Fixed tick-based simulation loop  
- Fully reproducible execution from initial state  
- Clear separation between simulation state and observers  

---

### Time-Travel Debugging (Core Capability)

- Snapshot of simulation state stored at every tick  
- Enables:
  - Step forward  
  - Step backward  
  - Replay from initial state 
  - Jump to arbitrary tick

This allows debugging not only the current state, but also **past system behavior**.

---

### Multi-Threaded Architecture

- Dedicated simulation thread  
- Console / future GUI operate on snapshots only  
- Synchronization via `std::mutex` and `std::condition_variable`  

Guarantees:

- No external mutation of simulation state  
- Thread-safe access  
- Preservation of deterministic behavior  

---

### Multi-Agent Simulation & Coordination

- Multiple robots operate concurrently in a shared grid  
- Each robot plans independently using:
  - A*  
  - Dijkstra  
  - BFS  

#### Conflict Resolution

- Priority-based coordination mechanism  
- When conflicts occur:
  - Higher-priority robot proceeds  
  - Lower-priority robot waits or replans  

#### Challenges Addressed

- Coordinating independent agents in shared mutable state  
- Preventing inconsistent updates  
- Maintaining determinism under concurrency  

---

### Modular Robotics Framework

#### Robot Abstraction

- Sense -> Plan -> Act pipeline  
- `GridRobot` as a concrete implementation  

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


### Planner Strategy Pattern

Path planners implement the Strategy Pattern  
Planners compute paths without modifying simulation state  

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

### Grid-Based Simulation

- Static and dynamic obstacles  
- Robot positions and goals  
- 4-directional movement (up/down/left/right)  
- Real-time console rendering  

---

### Console Debugger

Commands:

- `n` - step forward  
- `b` - step backward  
- `r` - run continuously  
- `p` - pause  
- `j` - jump to specific chosen tick
- `q` - quit  

Each command operates on consistent snapshots, ensuring reliable navigation through simulation time.

---

## Debugging Capabilities

- Tick-based breakpoints  
- State-based breakpoints  
- Event-based triggers  

Example:

- break at tick = 100  
- break when robot.state == REPLANNING  
- break on ObstacleDetected  

---

## Simulation Tick Flow

Each simulation tick executes the following steps:

1. All robots perform `sense()`
2. Each robot computes a plan using its planner
3. Conflict resolution is applied
4. Actions are committed to the simulation state
5. A snapshot is stored

This guarantees:
- Deterministic ordering
- Consistent state transitions  

---

## Performance Benchmarking

Performance metrics:

- Tick execution time (ms)  
- Planner computation time  
- Number of processed actions per tick  
- Pathfinding complexity (e.g., nodes expanded)  
- Planner execution time per robot

---

## GUI (Planned)

- Dear ImGui + SDL2  
- Simulation control panel  
- Grid visualization  
- Robot inspector  
- Event log  
- Timeline slider  
- Breakpoint management UI  

- The current console renderer is a temporary debugging interface and is intentionally decoupled from the simulation core to allow future replacement with a full graphical frontend.

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

GUI interacts only through Engine Controller  
Core engine independently testable  
SnapshotManager ensures deterministic replay  

---

## Design Principles

- Determinism first – reproducibility is a core requirement  
- Separation of concerns – simulation vs control vs presentation  
- Immutability for safety – snapshots instead of shared mutation  
- Modularity – interchangeable planners and robot behaviors  

---

## Design Patterns

- **Strategy Pattern** – path planning algorithms  
- **State Machine** – robot lifecycle  
- **Factory Pattern** – robot / planner creation  
- **RAII** – safe resource management  

---

## Memory Management

- `std::unique_ptr` for ownership  
- `std::shared_ptr` where shared lifetime is required  
- No raw owning pointers  
- Thread-safe state access via controlled interfaces  

---

## Project Structure

```
robotics-sim-debugger/
│
├── include/
│   ├── core/
│   ├── robots/
│   ├── planners/
│   ├── controller/
|   └── ui/
|       ├── console/
|       └── debug/
│
├── src/
│   ├── core/
│   ├── robots/
│   ├── planners/
│   ├── controller/
|   └── ui/
|       ├── console/
│
├── gui/ (planned)
└── tests/
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

## Testing Strategy

- Unit tests (GoogleTest)  
- Snapshot consistency validation  
- Step forward/backward correctness  
- Deterministic execution verification
- Breakpoint behavior validation    

---

## Current Status

- Deterministic simulation loop   
- Snapshot-based step forward/backward   
- Multi-agent coordination  
- Console debugger  
- Breakpoint syste
- Benchmarking
- GUI console   

In progress:

- GUI (Dear ImGui + SDL2)  
- Performance instrumentation  

---

## Motivation

This project focuses on core systems engineering challenges, including:

- Concurrency  
- Determinism  
- Debugging complex systems  
- Modular architecture  

The goal is to build a robust, inspectable simulation engine, rather than a graphics-heavy application.