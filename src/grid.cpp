#include "motion_planner/grid.hpp"
#include <random>

motion_planner::Grid::Grid(int width, int height) : width_(width), height_(height), cells_(height, std::vector<int>(width, 0)) {}

bool motion_planner::Grid::isValid(int x, int y) const{
    return x >= 0 && y >= 0 && x<width_ && y<height_;
}

void motion_planner::Grid::setObstacle(int x, int y) {
    if (isValid(x, y)) {
        cells_[y][x] = 1;
    }
}

void motion_planner::Grid::clearCell(int x, int y) {
    if (isValid(x, y)) {
        cells_[y][x] = 0;
    }
}

bool motion_planner::Grid::isObstacle(int x, int y) const {
    if (isValid(x, y)) {
        return cells_[y][x] == 1;
    }
    return false;
}

std::vector<std::pair<int, int>> motion_planner::Grid::getNeighbors(int x, int y) {
std::vector<std::pair<int, int>> neighbors;
int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, -1, 1};
for (int i = 0; i<4; i++){
    if(isValid(x+dx[i], y+dy[i]))
        neighbors.emplace_back(x+dx[i], y+dy[i]);
}
return neighbors;
}
void motion_planner::Grid::randomize(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, 1);
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            cells_[y][x] = dist(rng);
        }
    }
}