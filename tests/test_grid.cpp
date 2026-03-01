// =============================================================================
// test_grid.cpp — Unit tests for the Grid class
// =============================================================================
//
// GTest quick reference:
//   EXPECT_EQ(a, b)   → passes if a == b  (non-fatal, test continues)
//   EXPECT_TRUE(x)    → passes if x is true
//   EXPECT_FALSE(x)   → passes if x is false
//   ASSERT_EQ(a, b)   → like EXPECT_EQ but STOPS the test on failure
//
//   We link GTest::gtest_main so we don't write main() ourselves.
// =============================================================================

#include <gtest/gtest.h>
#include "motion_planner/grid.hpp"

using namespace motion_planner;

// ─── 1. Constructor stores width and height correctly ─────────────────────────
TEST(GridTest, SizeIsCorrect) {
    Grid g(10, 7);
    EXPECT_EQ(g.width(),  10);
    EXPECT_EQ(g.height(),  7);
}

// ─── 2. isValid boundary checks ──────────────────────────────────────────────
TEST(GridTest, InvalidCoordsReturnFalse) {
    Grid g(10, 10);

    // Negative → invalid
    EXPECT_FALSE(g.isValid(-1,  0));
    EXPECT_FALSE(g.isValid( 0, -1));

    // Corners → valid
    EXPECT_TRUE(g.isValid(0, 0));
    EXPECT_TRUE(g.isValid(9, 9));

    // One past the boundary → invalid
    EXPECT_FALSE(g.isValid(10,  0));
    EXPECT_FALSE(g.isValid( 0, 10));
    EXPECT_FALSE(g.isValid(10, 10));
}

// ─── 3. Set / query / clear obstacle lifecycle ───────────────────────────────
TEST(GridTest, SetAndClearObstacle) {
    Grid g(10, 10);

    // Initially free
    EXPECT_FALSE(g.isObstacle(5, 5));

    // Set → obstacle
    g.setObstacle(5, 5);
    EXPECT_TRUE(g.isObstacle(5, 5));

    // Neighbour unaffected
    EXPECT_FALSE(g.isObstacle(4, 5));

    // Clear → free again
    g.clearCell(5, 5);
    EXPECT_FALSE(g.isObstacle(5, 5));

    // Out-of-bounds calls must not crash
    g.setObstacle(-1, -1);
    g.setObstacle(100, 100);
    EXPECT_FALSE(g.isObstacle(-1, -1));
}

// ─── 4. Corner has fewer neighbours ──────────────────────────────────────────
TEST(GridTest, NeighborsOnCorner) {
    Grid g(10, 10);

    // (0,0) 4-connected → 2 neighbours (right, down)
    auto n4 = g.getFreeNeighbors(0, 0, false);
    EXPECT_EQ(n4.size(), 2u);

    // (0,0) 8-connected → 3 neighbours (right, down, diagonal)
    auto n8 = g.getFreeNeighbors(0, 0, true);
    EXPECT_EQ(n8.size(), 3u);

    // Centre cell, 4-connected → 4
    EXPECT_EQ(g.getFreeNeighbors(5, 5, false).size(), 4u);

    // Centre cell, 8-connected → 8
    EXPECT_EQ(g.getFreeNeighbors(5, 5, true).size(), 8u);
}

// ─── 5. Obstacles are excluded from neighbour list ───────────────────────────
TEST(GridTest, NeighborsFilterObstacles) {
    Grid g(10, 10);

    // Block cell to the right of (5,5)
    g.setObstacle(6, 5);
    auto neighbors = g.getFreeNeighbors(5, 5, false);
    EXPECT_EQ(neighbors.size(), 3u);

    // Verify (6,5) is NOT in the list
    for (const auto& n : neighbors) {
        EXPECT_FALSE(n.x == 6 && n.y == 5)
            << "Obstacle (6,5) should not appear as a free neighbour";
    }
}

// ─── 6. addWall fills a rectangle ────────────────────────────────────────────
TEST(GridTest, AddWallFillsRectangle) {
    Grid g(20, 20);

    g.addWall(2, 2, 4, 4);

    // Every cell inside the wall must be an obstacle
    for (int y = 2; y <= 4; ++y)
        for (int x = 2; x <= 4; ++x)
            EXPECT_TRUE(g.isObstacle(x, y))
                << "Expected obstacle at (" << x << "," << y << ")";

    // Cells just outside must still be free
    EXPECT_FALSE(g.isObstacle(1, 2));
    EXPECT_FALSE(g.isObstacle(5, 2));
    EXPECT_FALSE(g.isObstacle(2, 1));
    EXPECT_FALSE(g.isObstacle(2, 5));
}

// ─── 7. randomize respects extreme densities ─────────────────────────────────
TEST(GridTest, RandomizeExtremes) {
    Grid g(10, 10);

    // density 1.0 → every cell is an obstacle
    g.randomize(1.0, 42);
    for (int y = 0; y < g.height(); ++y)
        for (int x = 0; x < g.width(); ++x)
            EXPECT_TRUE(g.isObstacle(x, y));

    // density 0.0 → every cell is free
    g.randomize(0.0, 42);
    for (int y = 0; y < g.height(); ++y)
        for (int x = 0; x < g.width(); ++x)
            EXPECT_FALSE(g.isObstacle(x, y));
}

// ─── 8. getIndex / getCoords round-trip ──────────────────────────────────────
TEST(GridTest, IndexCoordsRoundTrip) {
    Grid g(15, 10);

    // Every (x,y) must survive the round trip: coords → index → coords
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            int idx = g.getIndex(x, y);
            Point2D p = g.getCoords(idx);
            EXPECT_EQ(p.x, x);
            EXPECT_EQ(p.y, y);
        }
    }
}
