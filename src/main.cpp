/**
 * @file main.cpp
 * @brief Demo: runs A* and RRT side-by-side on the same obstacle grid.
 */
#include "motion_planner/astar.hpp"
#include "motion_planner/grid.hpp"
#include "motion_planner/rrt.hpp"
#include "motion_planner/visualizer.hpp"
#include <iostream>

int main() {
    using namespace motion_planner;

    // ─── Environment setup ──────────────────────────────────────────
    Grid grid(100, 100);
    grid.addWall(30, 0,  30, 70);   // Vertical wall
    grid.addWall(60, 30, 60, 99);   // Vertical wall
    grid.addWall(30, 70, 50, 70);   // Horizontal connector

    const Point2D start{5, 5};
    const Point2D goal{95, 95};
    Visualizer viz(grid, 6);  // 6 px/cell → 600×600 image

    // ─── A* (optimal shortest path) ─────────────────────────────────
    AStar astar(grid);
    const PlanResult astar_result = astar.plan(start, goal);

    if (astar_result.success)
        std::cout << "[A*]  Path found | Steps: " << astar_result.path.size()
                  << " | Cost: " << astar_result.cost << '\n';
    else
        std::cout << "[A*]  No path found.\n";

    viz.show(astar_result, start, goal, "A* Result");
    viz.save("results/astar_result.png", astar_result, start, goal);

    // ─── RRT (sampling-based exploration) ────────────────────────────
    RRT rrt(grid);
    const RRTResult rrt_result = rrt.plan(start, goal, 42);  // seed=42

    if (rrt_result.success)
        std::cout << "[RRT] Path found | Steps: " << rrt_result.path.size()
                  << " | Tree edges: " << rrt_result.tree_edges.size() << '\n';
    else
        std::cout << "[RRT] No path found.\n";

    viz.show(rrt_result, start, goal, "RRT Result");
    viz.save("results/rrt_result.png", rrt_result, start, goal);

    return 0;
}