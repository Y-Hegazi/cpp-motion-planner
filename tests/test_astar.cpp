// =============================================================================
// test_astar.cpp — Unit tests for the A* planner
// =============================================================================

#include <gtest/gtest.h>
#include "motion_planner/astar.hpp"

using namespace motion_planner;

// ─── 1. Finds a path on a completely empty grid ─────────────────────────────
TEST(AStarTest, FindsPathOnEmptyGrid) {
    Grid g(50, 50);
    AStar planner(g);

    PlanResult result = planner.plan({0, 0}, {49, 49});

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.cost, 0.0);
}

// ─── 2. Path starts at start and ends at goal ───────────────────────────────
TEST(AStarTest, PathEndpoints) {
    Grid g(20, 20);
    AStar planner(g);

    Point2D start{0, 0};
    Point2D goal{19, 19};
    PlanResult result = planner.plan(start, goal);

    ASSERT_TRUE(result.success);
    ASSERT_FALSE(result.path.empty());

    // First point must be start
    EXPECT_EQ(result.path.front().x, start.x);
    EXPECT_EQ(result.path.front().y, start.y);

    // Last point must be goal
    EXPECT_EQ(result.path.back().x, goal.x);
    EXPECT_EQ(result.path.back().y, goal.y);
}

// ─── 3. Returns failure when the goal is completely walled off ──────────────
TEST(AStarTest, FailsWhenGoalBlocked) {
    Grid g(10, 10);

    // Build an impenetrable wall around position (8,8)
    // Wall spans x=[7..9], y=7 (top), y=9 (bottom), x=7 (left), x=9 (right)
    for (int i = 7; i <= 9; ++i) {
        g.setObstacle(i, 7);  // top row
        g.setObstacle(i, 9);  // bottom row
        g.setObstacle(7, i);  // left column
        g.setObstacle(9, i);  // right column
    }

    AStar planner(g);
    PlanResult result = planner.plan({0, 0}, {8, 8});

    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.path.empty());
}

// ─── 4. Every step in the path is contiguous (max 1 cell in each axis) ──────
TEST(AStarTest, PathIsContiguous) {
    Grid g(30, 30);
    AStar planner(g);

    PlanResult result = planner.plan({0, 0}, {29, 29});
    ASSERT_TRUE(result.success);

    for (size_t i = 1; i < result.path.size(); ++i) {
        int dx = std::abs(result.path[i].x - result.path[i - 1].x);
        int dy = std::abs(result.path[i].y - result.path[i - 1].y);

        EXPECT_LE(dx, 1) << "Step " << i << " jumps " << dx << " cells in x";
        EXPECT_LE(dy, 1) << "Step " << i << " jumps " << dy << " cells in y";
        // At least one axis must change (no standing still)
        EXPECT_GT(dx + dy, 0) << "Step " << i << " doesn't move at all";
    }
}

// ─── 5. No step in the path lands on an obstacle ───────────────────────────
TEST(AStarTest, PathAvoidsObstacles) {
    Grid g(30, 30);
    // Place a vertical wall in the middle, with a gap
    for (int y = 0; y < 25; ++y) {
        g.setObstacle(15, y);
    }

    AStar planner(g);
    PlanResult result = planner.plan({0, 0}, {29, 29});
    ASSERT_TRUE(result.success);

    for (const auto& p : result.path) {
        EXPECT_FALSE(g.isObstacle(p.x, p.y))
            << "Path goes through obstacle at (" << p.x << "," << p.y << ")";
    }
}

// ─── 6. Different heuristics all find a valid path ──────────────────────────
TEST(AStarTest, AllHeuristicsWork) {
    Grid g(20, 20);
    Point2D start{0, 0};
    Point2D goal{19, 19};

    for (auto h : {Heuristic::MANHATTAN, Heuristic::EUCLIDEAN, Heuristic::DIAGONAL}) {
        AStar planner(g, h);
        PlanResult result = planner.plan(start, goal);

        EXPECT_TRUE(result.success) << "Heuristic failed to find path";
        EXPECT_GT(result.cost, 0.0);
        EXPECT_EQ(result.path.front().x, start.x);
        EXPECT_EQ(result.path.front().y, start.y);
        EXPECT_EQ(result.path.back().x, goal.x);
        EXPECT_EQ(result.path.back().y, goal.y);
    }
}
