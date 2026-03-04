# Robotics Simulation Debugger

**Event-Driven, Multi-Threaded Robotics Simulation Framework with Interactive Debugger (C++ / Linux)**

---

## Overview

**Robotics Simulation Debugger** is a modular, event-driven simulation framework written in modern C++20, designed to model robotic systems with clean architecture, concurrency, and interactive state inspection.

This repository currently includes a **console MVP** that demonstrates the core simulation engine, snapshot-based time control, multi-robot planning, and dynamic obstacles. The GUI and advanced features will follow in subsequent stages.

This project demonstrates:

- Advanced OOP design  
- Polymorphism & SOLID principles  
- Strategy & Observer patterns  
- Event-driven architecture  
- Multi-threaded simulation  
- Snapshot-based time travel (rewind)  
- Clean separation between engine and UI  
- Linux-based CMake project structure  
- Comprehensive unit testing  

---

## Key Features

### 1. Event-Driven Architecture

Robots, sensors, and planners communicate through a decoupled **EventBus system**:

- Publish/Subscribe model  
- Thread-safe event queue  
- Asynchronous dispatch support  
- Strong separation of concerns  

### 2. Multi-Threaded Simulation

Threads:

- Simulation Thread – advances world state  
- Event Dispatch Thread – processes event queue  
- GUI Thread (future) – renders state & handles user interaction  

Synchronization via:

- `std::mutex`  
- `std::condition_variable`  
- Thread-safe queues  

> The console MVP runs in a single thread for simplicity; multi-threading is planned for GUI integration.

### 3. Snapshot-Based Time Control

Each simulation tick stores a snapshot of:

- Robot states  
- Environment state  
- Tick counter  

This enables:

- Pause / run  
- Step forward / backward  
- Timeline navigation  

Snapshots are **immutable** and allow deterministic rewinds.

### 4. Modular Robotics Framework

- **Robot abstraction** – base class defines `sense()`, `plan()`, `act()`  
- **Planner interface** – Strategy pattern with `AStarPlanner`, `DijkstraPlanner`, etc.  
- **Sensor abstraction** – e.g., `LidarSensor`, `MockSensor`  

---

## Architecture Overview

```
GUI Layer (future)
   ↓
Engine Controller
   ↓
Core Simulation Engine
   ├── Robots
   ├── Planners
   ├── Sensors
   ├── EventBus
   ├── Snapshot Manager
   └── Simulation Loop
```

- The GUI interacts only through the **Engine Controller**  
- The **Core Engine** is headless and testable independently  

---

## Design Patterns Used

- Strategy (Planners)  
- Observer / Pub-Sub (EventBus)  
- State Machine (Robot states)  
- Factory (Robot / Planner creation)  
- RAII for resource management  

---

## Project Structure

```
robotics-sim-debugger/
│
├── CMakeLists.txt
├── README.md
├── include/
│   ├── core/
│   ├── robots/
│   ├── planners/
│   ├── sensors/
│   └── controller/
├── src/
│   ├── core/
│   ├── robots/
│   ├── planners/
│   ├── sensors/
│   ├── controller/
│   └── main.cpp
├── gui/                   # GUI (Dear ImGui + SDL2, future)
├── tests/                 # Unit tests (GoogleTest)
└── assets/demo_scenarios/ # Config / grid layouts
```

---

## Build Instructions (Linux)

Requirements:

- C++20 compatible compiler  
- CMake ≥ 3.16  
- SDL2 + Dear ImGui (for GUI future)  
- GoogleTest (for unit tests)  

Build & run console MVP:

```bash
mkdir build
cd build
cmake ..
make
./robotics_sim_debugger
```

Run tests:

```bash
ctest
```

---

## Testing Strategy

- Unit tests for planners  
- EventBus behavior validation  
- Snapshot consistency verification  
- Deterministic simulation tests  
- Mock sensors for isolation testing  

---

## Future Extensions

- GUI with interactive Grid view, Inspector panel, Event log  
- Multi-threaded Simulation Engine + Event Dispatcher  
- Plugin-based planner loading  
- Physics engine integration  
- Distributed simulation nodes  
- Serialization of replay sessions  
- Performance profiling module  
- ECS (Entity Component System) refactor experiment  

---

## Motivation

This project is a **systems-oriented robotics simulation tool** emphasizing architecture, extensibility, and debuggability rather than graphics-heavy visualization.

It reflects a focus on:

- Core software engineering  
- Modern C++ practices  
- Robust architecture  
- Practical systems design