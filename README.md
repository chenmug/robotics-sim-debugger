# Robotics Simulation Debugger  
## Deterministic Multi-Agent Engine with Time-Travel Debugging
### C++ / Linux — Personal Project

---

## Overview

A deterministic multi-agent simulation engine built in C++20 for debugging, inspection, and reproducibility of concurrent systems.

The engine supports step-by-step execution, full state replay, and time-travel debugging of multi-agent behavior in a controlled environment.

Instead of focusing on visualization, the project focuses on making the system **observable, reproducible, and easy to debug under concurrency**.

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

## Path Planning (Pluggable)

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

This ensures deterministic execution and consistent state transitions.

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

## Performance Benchmarking (Secondary Feature)

A benchmarking module evaluates path-planning algorithms under different grid configurations.

- Grid sizes: up to 80×80
- Obstacle density: 10%–30%
- Multiple randomized runs per configuration

Metrics:
- Nodes expanded
- Execution time

Average results showed ~60% reduction in node expansions for A* compared to BFS and Dijkstra.

---

## Future Work

- Optional GUI visualization (Dear ImGui)
- Extended debugging events system