/**
 * @file astar.cpp
 * @brief A* implementation with IndexedPriorityQueue for O(E log V) performance.
 */
#include "motion_planner/astar.hpp"
#include "motion_planner/indexed_priority_queue.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_planner {

AStar::AStar(const Grid& grid, Heuristic h)
    : grid_(grid), heuristic_(h) {}

PlanResult AStar::plan(Point2D start, Point2D goal) const {
    PlanResult result;

    if (!grid_.isValid(start.x, start.y) || !grid_.isValid(goal.x, goal.y))
        return result;
    if (grid_.isObstacle(start.x, start.y) || grid_.isObstacle(goal.x, goal.y))
        return result;

    const int total = grid_.width() * grid_.height();
    const int start_idx = grid_.getIndex(start.x, start.y);
    const int goal_idx  = grid_.getIndex(goal.x, goal.y);

    // Flat 1D arrays indexed by cell index for cache-friendly access
    std::vector<double> g(total, std::numeric_limits<double>::infinity());
    std::vector<int>    parent(total, -1);

    IndexedPriorityQueue open(total);
    g[start_idx] = 0.0;
    open.push(start_idx, computeH(start, goal));

    while (!open.empty()) {
        const int cur = open.pop();

        if (cur == goal_idx) {
            result.success = true;
            result.cost = g[cur];

            // Reconstruct path from goal back to start
            std::vector<Point2D> path;
            for (int c = cur; c != -1; c = parent[c])
                path.push_back(grid_.getCoords(c));
            std::reverse(path.begin(), path.end());
            result.path = std::move(path);
            return result;
        }

        const Point2D cur_pt = grid_.getCoords(cur);

        for (const auto& nb : grid_.getFreeNeighbors(cur_pt.x, cur_pt.y, true)) {
            const int nb_idx = grid_.getIndex(nb.x, nb.y);

            // Diagonal moves cost √2, orthogonal moves cost 1
            const int dx = std::abs(nb.x - cur_pt.x);
            const int dy = std::abs(nb.y - cur_pt.y);
            const double step = (dx + dy == 2) ? std::sqrt(2.0) : 1.0;
            const double new_g = g[cur] + step;

            if (new_g < g[nb_idx]) {
                g[nb_idx] = new_g;
                parent[nb_idx] = cur;
                const double f = new_g + computeH(nb, goal);

                if (open.contains(nb_idx))
                    open.decreaseKey(nb_idx, f);  // O(log n) — IPQ advantage
                else
                    open.push(nb_idx, f);
            }
        }
    }

    return result;  // No path found
}

double AStar::computeH(Point2D a, Point2D b) const {
    const int dx = std::abs(a.x - b.x);
    const int dy = std::abs(a.y - b.y);

    switch (heuristic_) {
        case Heuristic::MANHATTAN:
            return dx + dy;
        case Heuristic::EUCLIDEAN:
            return std::sqrt(dx * dx + dy * dy);  // dx*dx avoids slow std::pow
        case Heuristic::DIAGONAL:
            // Octile distance — admissible for 8-connected grids
            return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    }
    return 0.0;
}

} // namespace motion_planner