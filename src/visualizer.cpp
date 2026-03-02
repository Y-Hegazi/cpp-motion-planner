#include "motion_planner/visualizer.hpp"

namespace motion_planner {
    Visualizer::Visualizer(const Grid& grid, int cell_size) : grid_(grid), cell_size_(cell_size) {
        image_ = cv::Mat(grid.height() * cell_size, grid.width() * cell_size, CV_8UC3, FREE_CELL_COLOR);
    }

    void Visualizer::drawCell(int x, int y, cv::Scalar color) {
        int px = x * cell_size_;
        int py = y * cell_size_;
        cv::rectangle(image_,
            cv::Point(px, py),
            cv::Point(px + cell_size_ - 1, py + cell_size_ - 1),
            color, cv::FILLED);
    }

    void Visualizer::drawGrid() {
        for (int y = 0; y < grid_.height(); ++y) {
            for (int x = 0; x < grid_.width(); ++x) {
                if (grid_.isObstacle(x, y))
                    drawCell(x, y, OBSTACLE_COLOR);
            }
        }
    }

    void Visualizer::drawPath(const PlanResult& result) {
        for (const auto& p : result.path)
            drawCell(p.x, p.y, PATH_COLOR);
    }

    void Visualizer::drawPoint(Point2D p, cv::Scalar color, int radius) {
        int px = p.x * cell_size_ + cell_size_ / 2;
        int py = p.y * cell_size_ + cell_size_ / 2;
        cv::circle(image_, cv::Point(px, py), radius, color, cv::FILLED);
    }

    void Visualizer::renderScene(const PlanResult& result, Point2D start, Point2D goal) {
        // Reset canvas to free space before drawing new state
        image_.setTo(FREE_CELL_COLOR); 
        
        drawGrid();
        drawPath(result);
        drawPoint(start, START_CELL_COLOR);
        drawPoint(goal, GOAL_CELL_COLOR);
    }

    void Visualizer::show(const std::string& title) {
        cv::imshow(title, image_);
        cv::waitKey(0);
    }
    
    void Visualizer::show(const PlanResult& result, Point2D start, Point2D goal, const std::string& title) {
        renderScene(result, start, goal);
        cv::imshow(title, image_);
        cv::waitKey(0);
    }

    void Visualizer::save(const std::string& path, const PlanResult& result, Point2D start, Point2D goal) {
        renderScene(result, start, goal);
        cv::imwrite(path, image_);
    }

}
