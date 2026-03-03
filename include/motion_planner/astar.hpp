/**
 * @file astar.hpp
 * @brief A* shortest-path planner with pluggable heuristics.
 *
 * Uses an IndexedPriorityQueue for O(E log V) performance via in-place
 * decreaseKey, rather than the common O(E log E) approach with std::priority_queue.
 */
#pragma once

#include "grid.hpp"

namespace motion_planner {

/// Result of an A* search.
struct PlanResult {
    std::vector<Point2D> path;   ///< Ordered waypoints from start to goal.
    double cost   = 0.0;        ///< Total path cost.
    bool   success = false;     ///< True if a path was found.
};

/// Admissible heuristic functions for A*.
enum class Heuristic { MANHATTAN, EUCLIDEAN, DIAGONAL };

/**
 * @class AStar
 * @brief A* planner operating on a Grid.
 *
 * Key optimizations:
 * - Custom IndexedPriorityQueue with O(log n) decreaseKey.
 * - Flat 1D score arrays indexed by cell index for cache locality.
 * - Diagonal (Octile) heuristic by default — admissible for 8-connected grids.
 */
class AStar {
public:
    /// @param grid  Reference to the occupancy grid.
    /// @param h     Heuristic to use (default: DIAGONAL / Octile distance).
    AStar(const Grid& grid, Heuristic h = Heuristic::DIAGONAL);

    [[nodiscard]] PlanResult plan(Point2D start, Point2D goal) const;

private:
    const Grid& grid_;
    Heuristic heuristic_;

    [[nodiscard]] double computeH(Point2D a, Point2D b) const;
};

} // namespace motion_planner