/**
 * @file visualizer.hpp
 * @brief OpenCV-based visualization for Grid, A*, and RRT results.
 */
#pragma once

#include "grid.hpp"
#include "astar.hpp"
#include "rrt.hpp"
#include <opencv2/opencv.hpp>
#include <string>

/// @name Visualization Colors (BGR format)
/// @{
inline const cv::Scalar kFreeColor     = cv::Scalar(255, 255, 255);  ///< White
inline const cv::Scalar kStartColor    = cv::Scalar(255, 255, 0);    ///< Cyan
inline const cv::Scalar kGoalColor     = cv::Scalar(0, 0, 255);      ///< Red
inline const cv::Scalar kObstacleColor = cv::Scalar(50, 50, 50);     ///< Dark gray
inline const cv::Scalar kPathColor     = cv::Scalar(0, 255, 0);      ///< Green
inline const cv::Scalar kRRTTreeColor  = cv::Scalar(200, 180, 130);  ///< Light blue-gray
inline const cv::Scalar kRRTPathColor  = cv::Scalar(0, 165, 255);    ///< Orange
/// @}

namespace motion_planner {

/**
 * @class Visualizer
 * @brief Renders planning results onto a grid image using OpenCV.
 *
 * Overloaded show/save methods accept either PlanResult (A*) or RRTResult.
 */
class Visualizer {
public:
    /// @param grid      Reference to the occupancy grid.
    /// @param cell_size Pixels per grid cell (controls output image resolution).
    Visualizer(const Grid& grid, int cell_size = 4);

    void drawGrid();
    void drawPath(const PlanResult& result);
    void drawPoint(Point2D p, cv::Scalar color, int radius = 4);

    /// @name A* Visualization
    /// @{
    void show(const std::string& title = "Motion Planner");
    void show(const PlanResult& result, Point2D start, Point2D goal,
              const std::string& title = "A* Result");
    void save(const std::string& path, const PlanResult& result,
              Point2D start, Point2D goal);
    /// @}

    /// @name RRT Visualization
    /// @{
    void drawRRTTree(const RRTResult& result);
    void drawRRTPath(const RRTResult& result);
    void show(const RRTResult& result, Point2D start, Point2D goal,
              const std::string& title = "RRT Result");
    void save(const std::string& path, const RRTResult& result,
              Point2D start, Point2D goal);
    /// @}

private:
    const Grid& grid_;
    int cell_size_;
    cv::Mat image_;

    void drawCell(int x, int y, cv::Scalar color);
    void renderScene(const PlanResult& result, Point2D start, Point2D goal);
    void renderScene(const RRTResult& result, Point2D start, Point2D goal);
};

} // namespace motion_planner
