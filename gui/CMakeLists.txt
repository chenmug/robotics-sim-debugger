cmake_minimum_required(VERSION 3.16)

project(sim_gui LANGUAGES CXX)

# =========================
# C++ Standard
# =========================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================
# Dependencies
# =========================
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

# =========================
# GUI Executable
# =========================
add_executable(sim_gui
    main.cpp

    # =========================
    # Dear ImGui Core
    # =========================
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/imgui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/imgui_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/imgui_tables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/imgui_widgets.cpp

    # =========================
    # Dear ImGui Backends
    # =========================
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/backends/imgui_impl_sdl2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/backends/imgui_impl_opengl3.cpp
)

# =========================
# Include Directories
# =========================
target_include_directories(sim_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/backends
)

# =========================
# Link Libraries
# =========================
target_link_libraries(sim_gui PRIVATE
    OpenGL::GL
    SDL2::SDL2
)

# =========================
# Dear ImGui Feature Flags
# =========================
target_compile_definitions(sim_gui PRIVATE
    IMGUI_ENABLE_DOCKING
    IMGUI_ENABLE_VIEWPORTS
)

# =========================
# Linux system libs
# =========================
target_link_libraries(sim_gui PRIVATE dl pthread)