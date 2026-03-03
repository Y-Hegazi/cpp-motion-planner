#include <gtest/gtest.h>
#include "motion_planner/rrt.hpp"
#include "motion_planner/grid.hpp"

using namespace motion_planner;

class RRTTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Small grid for fast tests
        grid = std::make_unique<Grid>(20, 20);
    }
    
    std::unique_ptr<Grid> grid;
};

TEST_F(RRTTest, FindsPathOnEmptyGrid) {
    RRT rrt(*grid);
    Point2D start{0, 0};
    Point2D goal{19, 19};
    
    // Seed = 42 for reproducible test
    RRTResult result = rrt.plan(start, goal, 42);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    
    // Start node is implicitly first in path (though technically RRT path 
    // Reconstruction includes the start because the tree root is start)
    EXPECT_EQ(result.path.front().x, start.x);
    EXPECT_EQ(result.path.front().y, start.y);
    
    // End node should be close to goal (within goal threshold)
    double dx = result.path.back().x - goal.x;
    double dy = result.path.back().y - goal.y;
    double dist_sq = dx * dx + dy * dy;
    EXPECT_LE(dist_sq, 3.0 * 3.0); // Default threshold is 3.0
}

TEST_F(RRTTest, FailsWhenGoalBlocked) {
    // Completely cut off the bottom half of the grid
    grid->addWall(0, 15, 19, 15);
    
    RRT rrt(*grid);
    Point2D start{0, 0};
    Point2D goal{19, 19};
    
    // Set max iterations low to fail quickly
    RRTConfig config;
    config.max_iterations = 200;
    RRT quick_rrt(*grid, config);
    
    RRTResult result = quick_rrt.plan(start, goal, 42);
    
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.path.empty());
}

TEST_F(RRTTest, ReproducibleWithSameSeed) {
    RRT rrt(*grid);
    Point2D start{2, 2};
    Point2D goal{15, 15};
    
    // Two plans with the exact same seed should produce the exact same tree and path
    RRTResult result1 = rrt.plan(start, goal, 12345);
    RRTResult result2 = rrt.plan(start, goal, 12345);
    
    EXPECT_TRUE(result1.success);
    EXPECT_TRUE(result2.success);
    
    EXPECT_EQ(result1.path.size(), result2.path.size());
    EXPECT_EQ(result1.tree_edges.size(), result2.tree_edges.size());
    
    for (size_t i = 0; i < result1.path.size(); ++i) {
        EXPECT_EQ(result1.path[i].x, result2.path[i].x);
        EXPECT_EQ(result1.path[i].y, result2.path[i].y);
    }
}

TEST_F(RRTTest, InvalidPointsReturnFalse) {
    RRT rrt(*grid);
    
    // Start out of bounds
    Point2D invalid_start{25, 25};
    Point2D goal{10, 10};
    RRTResult result1 = rrt.plan(invalid_start, goal);
    EXPECT_FALSE(result1.success);
    
    // Goal out of bounds
    Point2D start{0, 0};
    Point2D invalid_goal{-5, 5};
    RRTResult result2 = rrt.plan(start, invalid_goal);
    EXPECT_FALSE(result2.success);
}
