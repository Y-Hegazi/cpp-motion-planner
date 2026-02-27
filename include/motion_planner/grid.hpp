#pragma once

#include <vector>
namespace motion_planner{
    class Grid{
        public:
            Grid(int width, int height);
            void setObstacle(int x, int y);
            void clearCell(int x, int y);
            bool isObstacle(int x, int y) const;
            bool isValid(int x, int y) const;
            std::vector<std::pair<int, int>> getNeighbors(int x, int y);
            void randomize(unsigned seed = 0);
            int width() const {return width_ ;};
            int height() const {return height_ ;};


        private:
            int width_;
            int height_;
            std::vector<std::vector<int>> cells_; // y,x
    };
}