cmake_minimum_required(VERSION 3.14)
project(motion_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

add_library(motion_planner_lib STATIC
src/grid.cpp
src/astar.cpp
src/visualizer.cpp
src/rrt.cpp)

target_include_directories(motion_planner_lib PUBLIC include/)

target_link_libraries(motion_planner_lib PRIVATE ${OpenCV_LIBS})

add_executable(motion_planner src/main.cpp)
target_link_libraries(motion_planner PRIVATE motion_planner_lib)

option(BUILD_TESTS "BUILD_TESTS" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()