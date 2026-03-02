#include "motion_planner/astar.hpp"
#include "motion_planner/indexed_priority_queue.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace motion_planner {

AStar::AStar(const Grid& grid, Heuristic h) : grid_(grid), heuristic_(h) {}

PlanResult AStar::plan(Point2D start, Point2D goal) const {
    PlanResult result;
    result.success = false;

    if (!grid_.isValid(start.x, start.y) || !grid_.isValid(goal.x, goal.y))
        return result;
    if (grid_.isObstacle(start.x, start.y) || grid_.isObstacle(goal.x, goal.y))
        return result;

    int total_cells = grid_.width() * grid_.height();
    int start_idx = grid_.getIndex(start.x, start.y);
    int goal_idx  = grid_.getIndex(goal.x, goal.y);

    std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());
    std::vector<int>    came_from(total_cells, -1);

    IndexedPriorityQueue open_set(total_cells);

    g_score[start_idx] = 0.0;
    open_set.push(start_idx, computeH(start, goal));

    while (!open_set.empty()) {
        int current_idx = open_set.pop();
        Point2D current = grid_.getCoords(current_idx);

        // Goal reached — reconstruct path
        if (current_idx == goal_idx) {
            result.success = true;
            result.cost = g_score[current_idx];

            std::vector<Point2D> path;
            int curr = current_idx;
            while (curr != -1) {
                path.push_back(grid_.getCoords(curr));
                curr = came_from[curr];
            }
            std::reverse(path.begin(), path.end());
            result.path = path;
            return result;
        }

        // Expand neighbors
        for (const auto& neighbor : grid_.getFreeNeighbors(current.x, current.y, true)) {
            int neighbor_idx = grid_.getIndex(neighbor.x, neighbor.y);

            int dx = std::abs(neighbor.x - current.x);
            int dy = std::abs(neighbor.y - current.y);
            double step_cost = (dx + dy == 2) ? std::sqrt(2.0) : 1.0;
            double new_g = g_score[current_idx] + step_cost;

            if (new_g < g_score[neighbor_idx]) {
                g_score[neighbor_idx] = new_g;
                came_from[neighbor_idx] = current_idx;
                double f = new_g + computeH(neighbor, goal);

                if (open_set.contains(neighbor_idx))
                    open_set.decreaseKey(neighbor_idx, f);
                else
                    open_set.push(neighbor_idx, f);
            }
        }
    }

    return result;
}

double AStar::computeH(Point2D a, Point2D b) const {
    switch (heuristic_) {
        case Heuristic::MANHATTAN:
            return std::abs(a.x - b.x) + std::abs(a.y - b.y);
        case Heuristic::EUCLIDEAN:
            return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
        case Heuristic::DIAGONAL: {
            int dx = std::abs(a.x - b.x);
            int dy = std::abs(a.y - b.y);
            return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
        }
    }
    return 0.0;
}

}