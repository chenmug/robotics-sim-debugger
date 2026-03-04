cmake_minimum_required(VERSION 3.16)
project(robotics_sim_debugger VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# include directories
include_directories(include)

# source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# executable
add_executable(robotics_sim_debugger ${SOURCES})