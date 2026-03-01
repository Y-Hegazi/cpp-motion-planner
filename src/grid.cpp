#include "motion_planner/grid.hpp"
#include <algorithm>
#include <iostream>
#include <array>

using std::vector;

constexpr std::array<motion_planner::Point2D, 8> deltas = {{
    {0, 1}, {0, -1}, {1, 0}, {-1, 0}, // 4-connected (orthogonal)
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1} // 8-connected (diagonal)
}};

motion_planner::Grid::Grid(int width, int height) : width_(width), height_(height), cells_(height*width, CellType::FREE) {}

bool motion_planner::Grid::isValid(int x, int y) const{
    return x >= 0 && y >= 0 && x<width_ && y<height_;
}

void motion_planner::Grid::setObstacle(int x, int y) {
    if (isValid(x, y)) {
        cells_[getIndex(x,y)] = CellType::OBSTACLE;
    }
}

void motion_planner::Grid::clearCell(int x, int y) {
    if (isValid(x, y)) {
        cells_[getIndex(x,y)] = CellType::FREE;
    }
}

bool motion_planner::Grid::isObstacle(int x, int y) const {
    if (isValid(x, y)) {
        return cells_[getIndex(x,y)] == CellType::OBSTACLE;
    }
    return false;
}
    
vector<motion_planner::Point2D> motion_planner::Grid::getFreeNeighbors(int x, int y, bool eight_connected) const {
vector<motion_planner::Point2D> neighbors;
const int limit = eight_connected ? 8 : 4;
neighbors.reserve(limit);
for (int i = 0; i < limit; i++){
    int nx = x+deltas[i].x;
    int ny = y+deltas[i].y;
    if (isValid(nx, ny) && !isObstacle(nx, ny)){
        neighbors.emplace_back(motion_planner::Point2D{nx, ny});
    }
}   
return neighbors;
}

void motion_planner::Grid::addWall(int x0, int y0, int x1, int y1){
    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    x1 = std::min(width_-1, x1);
    y1 = std::min(height_-1, y1);
    
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);
    
    for (int x = x0; x <= x1; x++) {
        for (int y = y0; y <= y1; y++) {
            setObstacle(x, y);
        }
    }
}

void motion_planner::Grid::randomize(double obstacle_density, unsigned seed) {
    rng_.seed(seed);
    obstacle_density = std::clamp(obstacle_density, 0.0, 1.0); // Clamp the obstacle density in case the user was an idiot.
    std::bernoulli_distribution dist(obstacle_density);

    for (auto &cell : cells_) {
        cell = dist(rng_) ? CellType::OBSTACLE : CellType::FREE;
    }
}

void motion_planner::Grid::print() const {
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            std::cout << static_cast<int>(cells_[getIndex(x,y)]) << " ";
        }
        std::cout << std::endl;
    }
}
