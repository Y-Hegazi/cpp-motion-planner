find_package(GTest REQUIRED)

add_executable(test_grid test_grid.cpp)
target_link_libraries(test_grid GTest::gtest_main motion_planner_lib)

# add_executable(test_astar test_astar.cpp)
# target_link_libraries(test_astar GTest::gtest_main motion_planner_lib)

# add_executable(test_rrt test_rrt.cpp)
# target_link_libraries(test_rrt GTest::gtest_main motion_planner_lib)

include(GoogleTest)
gtest_discover_tests(test_grid)
# gtest_discover_tests(test_astar)
# gtest_discover_tests(test_rrt)

