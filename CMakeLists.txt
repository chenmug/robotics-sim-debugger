cmake_minimum_required(VERSION 3.16)
project(robotics_sim_debugger VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)

# =========================================================
# Core sources
# =========================================================
file(GLOB_RECURSE CORE_SOURCES
    "src/core/*.cpp"
    "src/planners/*.cpp"
    "src/robots/*.cpp"
    "src/sensors/*.cpp"
    "src/controller/*.cpp"
    "src/ui/**/*.cpp"
    "src/benchmarks/*.cpp"
)

add_library(robotics_sim_lib ${CORE_SOURCES})

target_include_directories(robotics_sim_lib PUBLIC include)

# =========================================================
# Main executable
# =========================================================
add_executable(robotics_sim_debugger src/main.cpp)
target_link_libraries(robotics_sim_debugger robotics_sim_lib)

# =========================================================
# Benchmark executable
# =========================================================
add_executable(robotics_benchmark src/benchmark_main.cpp)
target_link_libraries(robotics_benchmark robotics_sim_lib)

# =========================================================
# Tests
# =========================================================
enable_testing()
add_subdirectory(tests)