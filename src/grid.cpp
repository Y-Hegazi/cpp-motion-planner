/**
 * @file grid.cpp
 * @brief Grid implementation — obstacle management, neighbor lookup, and wall generation.
 */
#include "motion_planner/grid.hpp"
#include <algorithm>
#include <array>
#include <iostream>

using std::vector;

namespace motion_planner {

/// Direction offsets: first 4 = orthogonal, next 4 = diagonal.
constexpr std::array<Point2D, 8> kDeltas = {{
    {0, 1}, {0, -1}, {1, 0}, {-1, 0},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
}};

Grid::Grid(int width, int height)
    : width_(width), height_(height),
      cells_(width * height, CellType::FREE) {}

bool Grid::isValid(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
}

void Grid::setObstacle(int x, int y) {
    if (isValid(x, y))
        cells_[getIndex(x, y)] = CellType::OBSTACLE;
}

void Grid::clearCell(int x, int y) {
    if (isValid(x, y))
        cells_[getIndex(x, y)] = CellType::FREE;
}

bool Grid::isObstacle(int x, int y) const {
    return isValid(x, y) && cells_[getIndex(x, y)] == CellType::OBSTACLE;
}

vector<Point2D> Grid::getFreeNeighbors(int x, int y, bool eight_connected) const {
    const int limit = eight_connected ? 8 : 4;
    vector<Point2D> neighbors;
    neighbors.reserve(limit);

    for (int i = 0; i < limit; ++i) {
        int nx = x + kDeltas[i].x;
        int ny = y + kDeltas[i].y;
        if (isValid(nx, ny) && !isObstacle(nx, ny))
            neighbors.push_back({nx, ny});
    }
    return neighbors;
}

void Grid::addWall(int x0, int y0, int x1, int y1) {
    // Clamp to grid and normalize direction
    x0 = std::max(0, x0);  y0 = std::max(0, y0);
    x1 = std::min(width_ - 1, x1);  y1 = std::min(height_ - 1, y1);
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);

    for (int x = x0; x <= x1; ++x)
        for (int y = y0; y <= y1; ++y)
            setObstacle(x, y);
}

void Grid::randomize(double obstacle_density, unsigned seed) {
    rng_.seed(seed);
    obstacle_density = std::clamp(obstacle_density, 0.0, 1.0);
    std::bernoulli_distribution dist(obstacle_density);

    for (auto& cell : cells_)
        cell = dist(rng_) ? CellType::OBSTACLE : CellType::FREE;
}

void Grid::print() const {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x)
            std::cout << static_cast<int>(cells_[getIndex(x, y)]) << ' ';
        std::cout << '\n';
    }
}

} // namespace motion_planner
