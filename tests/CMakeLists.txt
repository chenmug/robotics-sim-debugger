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