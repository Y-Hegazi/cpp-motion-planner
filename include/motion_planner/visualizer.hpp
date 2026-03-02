#pragma once
#include "grid.hpp"
#include "astar.hpp"
#include <opencv2/opencv.hpp>
#include <string>

// Colors for the grid (BGR format)
inline const cv::Scalar FREE_CELL_COLOR = cv::Scalar(255, 255, 255); // white
inline const cv::Scalar START_CELL_COLOR = cv::Scalar(255, 255, 0);  // yellow
inline const cv::Scalar GOAL_CELL_COLOR = cv::Scalar(0, 0, 255);     // red
inline const cv::Scalar OBSTACLE_COLOR = cv::Scalar(50, 50, 50);     // dark gray
inline const cv::Scalar PATH_COLOR = cv::Scalar(0, 255, 0);          // green

namespace motion_planner {

class Visualizer {
public:
    Visualizer(const Grid& grid, int cell_size = 4);

    void drawGrid();
    void drawPath(const PlanResult& result);
    void drawPoint(Point2D p, cv::Scalar color, int radius = 4);
    void show(const std::string& title = "Motion Planner"); // Just show the raw grid
    void show(const PlanResult& result, Point2D start, Point2D goal, const std::string& title = "Motion Planner");
    void save(const std::string& path, const PlanResult& result, Point2D start, Point2D goal);

private:
    const Grid& grid_;
    int cell_size_;
    cv::Mat image_;

    void drawCell(int x, int y, cv::Scalar color);
    void renderScene(const PlanResult& result, Point2D start, Point2D goal);
};

}
