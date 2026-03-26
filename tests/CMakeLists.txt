# Fetch GoogleTest automatically
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

FetchContent_MakeAvailable(googletest)


# ===============================
# A* tests
# ===============================
add_executable(test_astar test_astar.cpp)

target_link_libraries(test_astar
    robotics_sim_lib
    gtest
    gtest_main
)

add_test(NAME AStarTests COMMAND test_astar)


# ===============================
# Dijkstra tests
# ===============================
add_executable(test_dijkstra test_dijkstra.cpp)

target_link_libraries(test_dijkstra
    robotics_sim_lib
    gtest
    gtest_main
)

add_test(NAME DijkstraTests COMMAND test_dijkstra)


# ===============================
# BFS tests
# ===============================
add_executable(test_bfs test_bfs.cpp)

target_link_libraries(test_bfs
    robotics_sim_lib
    gtest
    gtest_main
)

add_test(NAME BFSTests COMMAND test_bfs)


# ===============================
# SnapshotManager tests
# ===============================
add_executable(test_snapshots test_snapshots.cpp)

target_link_libraries(test_snapshots
    robotics_sim_lib
    gtest
    gtest_main
)

add_test(NAME SnapshotManagerTests COMMAND test_snapshots)


# ===============================
# EngineController tests
# ===============================
add_executable(test_controller test_controller.cpp)

target_link_libraries(test_controller
    robotics_sim_lib
    gtest
    gtest_main
)

add_test(NAME EngineControllerTests COMMAND test_controller)


# ===============================
# BreakpointManager tests
# ===============================
add_executable(test_breakpoints test_breakpoints.cpp)

target_link_libraries(test_breakpoints
    robotics_sim_lib
    gtest
    gtest_main
)

add_test(NAME BreakpointManagerTests COMMAND test_breakpoints)