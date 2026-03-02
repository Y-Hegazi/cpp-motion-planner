#pragma once

#include "grid.hpp"

namespace motion_planner{
struct PlanResult{
    std::vector<Point2D> path;
    double cost;
    bool success;
};

enum class Heuristic { MANHATTAN, EUCLIDEAN, DIAGONAL };

class AStar {
public:
    AStar(const Grid& grid, Heuristic h = Heuristic::DIAGONAL);
    [[nodiscard]]PlanResult plan(Point2D start, Point2D goal) const;

private:
    const Grid& grid_;
    Heuristic heuristic_;
    [[nodiscard]]double computeH(Point2D a, Point2D b) const;
};

}