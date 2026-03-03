/**
 * @file rrt.hpp
 * @brief Rapidly-exploring Random Tree (RRT) planner.
 *
 * Sampling-based planner that grows a tree of collision-free edges from the
 * start toward randomly sampled points. Unlike A*, RRT works in continuous
 * space and scales to high-dimensional configuration spaces.
 *
 * Key optimizations:
 * - Dynamic KD-Tree for O(log n) nearest-neighbor queries (vs O(n) brute-force).
 * - Squared-distance comparisons to eliminate sqrt calls in the inner loop.
 * - Goal-biased sampling to steer exploration toward the target.
 */
#pragma once

#include "grid.hpp"
#include <random>
#include <utility>
#include <vector>

namespace motion_planner {

/// Result returned by the RRT planner.
struct RRTResult {
    std::vector<Point2D> path;                            ///< Waypoints from start to goal.
    std::vector<std::pair<Point2D, Point2D>> tree_edges;  ///< Every (parent→child) edge built.
    bool success = false;                                 ///< True if a path was found.
};

/// Tuning knobs for the RRT planner.
struct RRTConfig {
    int    max_iterations = 5000;  ///< Max random samples before giving up.
    double step_size      = 3.0;   ///< Max extension distance per iteration.
    double goal_threshold = 3.0;   ///< Distance to goal that counts as "reached".
    double goal_bias      = 0.05;  ///< Probability [0,1] of sampling the goal directly.
};

/**
 * @class RRT
 * @brief Rapidly-exploring Random Tree planner on a 2D Grid.
 *
 * Internally builds a flat `std::vector<RRTNode>` as the tree and a dynamic
 * KD-Tree (also flat-vector based) for O(log n) nearest-neighbor lookups.
 */
class RRT {
public:
    /// @param grid   Reference to the occupancy grid.
    /// @param config Tuning parameters (defaults are reasonable for 100×100 grids).
    explicit RRT(const Grid& grid, RRTConfig config = {});

    /// @brief Plan a path from start to goal.
    /// @param seed  RNG seed (0 = non-deterministic, >0 = reproducible).
    [[nodiscard]] RRTResult plan(Point2D start, Point2D goal, unsigned seed = 0) const;

private:
    const Grid& grid_;
    RRTConfig config_;
    mutable std::mt19937 rng_;  ///< Mutable: RNG state changes inside const plan().

    /// @brief Checks if the straight-line edge from (x0,y0) to (x1,y1) is obstacle-free.
    [[nodiscard]] bool isEdgeFree(double x0, double y0, double x1, double y1) const;
};

} // namespace motion_planner
