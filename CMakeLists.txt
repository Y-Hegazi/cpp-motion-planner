cmake_minimum_required(VERSION 3.14)
project(motion_planner
    VERSION   1.0.0
    LANGUAGES CXX
    DESCRIPTION "A* and RRT motion planning on 2D occupancy grids")

# ─── Compiler settings ───────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)       # No GNU extensions — strict ISO C++17

# ─── Dependencies ─────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)

# ─── Library ──────────────────────────────────────────────────────────────────
add_library(motion_planner_lib STATIC
    src/grid.cpp
    src/astar.cpp
    src/rrt.cpp
    src/visualizer.cpp)

target_include_directories(motion_planner_lib PUBLIC include/)

# PUBLIC because visualizer.hpp exposes cv::Scalar / cv::Mat in its interface
target_link_libraries(motion_planner_lib PUBLIC ${OpenCV_LIBS})

# ─── Executable ───────────────────────────────────────────────────────────────
add_executable(motion_planner src/main.cpp)
target_link_libraries(motion_planner PRIVATE motion_planner_lib)

# ─── Tests ────────────────────────────────────────────────────────────────────
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()