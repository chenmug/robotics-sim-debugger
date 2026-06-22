# Robotics Simulation Debugger  
### C++ / Linux - Personal Project  

Deterministic multi-agent simulation engine with time-travel debugging

---

## Overview

A deterministic multi-agent simulation engine built in C++20 for debugging, inspection, and reproducibility of concurrent systems.

The engine supports step-by-step execution, full state replay, and time-travel debugging of multi-agent behavior in a controlled environment.

Instead of focusing on visualization, the project focuses on making the system **observable, reproducible, and easy to debug under concurrency**.

---

## Key Highlights

- Deterministic execution with full replay capability
- Snapshot-based time-travel debugging
- Multi-agent grid simulation with conflict resolution
- Pluggable path-planning algorithms (BFS, Dijkstra, A*)
- Benchmarking for performance evaluation

---

### System Guarantees

- Deterministic replay from any saved snapshot
- Identical inputs always produce identical execution traces
- No shared mutable state between simulation components

---

## Motivation

Debugging concurrent systems is hard due to:

- Non-deterministic execution order  
- Race conditions in shared state  
- Complex interactions between multiple agents  

This project explores how to turn such a system into a fully reproducible simulation engine that allows:

- Deterministic execution  
- Replay of past runs  
- Step-level debugging at tick granularity  

---

## Core Features

### Deterministic Simulation Engine
- Tick-based execution model  
- Fully reproducible runs from the same initial state  
- Clear separation between simulation state and execution logic  

---

### Time-Travel Debugging
- Snapshot of the full simulation state at every tick  
- Step forward and backward through execution history  
- Breakpoints based on ticks or state conditions  
- Replay and inspect past states  

---

### Multi-Agent System
- Multiple robots running in a shared grid  
- Each agent plans independently  
- Built-in conflict resolution for simultaneous actions  

---

### Multi-Threaded Architecture
- Simulation runs in a dedicated thread, separate from the control interface  
- Uses `std::mutex` and `std::condition_variable` for synchronization  
- Snapshot-based state access ensures deterministic behavior  

---

### Conflict Resolution
- Priority-based ordering of actions  
- Deterministic handling of collisions and conflicts  
- Guarantees consistent results across runs  

---

### Modular Design
- Robot abstraction (sense → plan → act)  
- Pluggable path-planning algorithms  
- Decoupled simulation core  

---

## Path Planning

Implemented algorithms:

- BFS  
- Dijkstra  
- A*  

All used interchangeably within the same engine.

---

## Grid Simulation

- 4-direction movement (up, down, left, right)  
- Random obstacle generation  
- Start/goal validation to ensure solvable grids  
- Deterministic environment generation  

---

## Debug Interface (Console)

- Step forward / backward  
- Pause / resume execution  
- Jump to any tick  
- Inspect robots and simulation state  

---

## Execution Model

Each simulation tick:

1. Robots sense the environment  
2. Planners compute actions  
3. Conflicts are resolved  
4. Actions are applied  
5. A snapshot is saved  

This pipeline guarantees deterministic execution and reproducible state transitions.

---

## Architecture

```text
Engine Controller
      ↓
Deterministic Simulation Engine
      ↓
Simulation Core
├── Simulation Loop
├── Robots
├── Planners
├── Snapshot Manager
└── Conflict Resolution
```

The engine is fully decoupled from any UI layer and designed for testability and reproducibility. 

---

## Design Principles

- Determinism as a core requirement
- Clear separation between simulation and control logic
- Snapshot-based state instead of shared mutable state
- Modular and extensible architecture

---

## Tech Stack

- C++20
- Linux
- Multithreading (mutex, condition_variable)
- GoogleTest
- Graph algorithms

---

## Testing Strategy

The system is validated for deterministic behavior and correctness of time-travel execution

- Unit tests (GoogleTest) for core modules
- Snapshot-based consistency validation across ticks
- Verification of backward/forward replay correctness
- Breakpoint and event-trigger correctness validation

---

## Build Instructions (Linux)

### Requirements

- C++20 compatible compiler
- CMake ≥ 3.16

### Build

```bash
mkdir build
cd build
cmake ..
make
```

### Run simulation:

```bash
./robotics_sim_debugger
```

### Run benchmark:

```bash
./robotics_benchmark
```

### Run tests:

```bash
ctest
```

---

## Performance Benchmarking (Secondary Feature)

A benchmarking module evaluates path-planning algorithms under different grid configurations.

- Grid sizes: up to 80×80
- Obstacle density: 10%–30%
- Multiple randomized runs per configuration

Metrics:
- Nodes expanded
- Execution time

### Benchmark results:

- ~60% reduction in node expansions for A* compared to BFS and Dijkstra.
- Consistently lower execution time across all tested configurations.

---

## Future Work

- GUI visualization using Dear ImGui
- Extended event-based debugging system
- Performance profiling and instrumentation improvements