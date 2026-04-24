cmake_minimum_required(VERSION 3.16)
project(robotics_sim_debugger VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)

# include directories
include_directories(include)

# source files
file(GLOB_RECURSE SOURCES
    "src/core/*.cpp"
    "src/planners/*.cpp"
    "src/robots/*.cpp"
    "src/sensors/*.cpp"
    "src/controller/*.cpp"
    "src/ui/**/*.cpp"
)

# library with all core code
add_library(robotics_sim_lib ${SOURCES})

# main executable
add_executable(robotics_sim_debugger src/main.cpp)

target_link_libraries(robotics_sim_debugger robotics_sim_lib)

# enable testing
enable_testing()

# add tests folder
add_subdirectory(tests)

# add gui folder
add_subdirectory(gui)