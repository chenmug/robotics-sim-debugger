cmake_minimum_required(VERSION 3.16)

project(sim_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SDL2 REQUIRED)

add_executable(sim_gui
    main.cpp
)

target_link_libraries(sim_gui PRIVATE SDL2::SDL2)