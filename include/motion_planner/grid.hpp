#pragma once

#include <vector>
#include <random>
#include <cstdint>

namespace motion_planner{
    struct Point2D{
        int x;
        int y;
    };

    enum class CellType : std::uint8_t{
        FREE = 0,
        OBSTACLE = 1,
        START = 2,
        GOAL = 3,
        PATH = 4,
        VISITED = 5
};


    class Grid{
        public:
            Grid(int width, int height);
            void setObstacle(int x, int y);
            void clearCell(int x, int y);
            [[nodiscard]] bool isObstacle(int x, int y) const;
            [[nodiscard]] bool isValid(int x, int y) const;
            [[nodiscard]] std::vector<Point2D> getFreeNeighbors(int x, int y, bool eight_connected = false) const;
            void randomize(double obstacle_density, unsigned seed = 0);
            [[nodiscard]] int width() const {return width_ ;}
            [[nodiscard]] int height() const {return height_ ;}
            void print() const;
            void addWall(int x0, int y0, int x1, int y1);
            [[nodiscard]] int getIndex(int x, int y) const {return y*width_+x;}
            [[nodiscard]] Point2D getCoords(int index) const {return {index%width_, index/width_};}

        private:
            int width_;
            int height_;
            std::vector<CellType> cells_;
            std::mt19937 rng_;
    };
}