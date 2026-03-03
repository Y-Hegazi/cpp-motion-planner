/**
 * @file visualizer.cpp
 * @brief OpenCV rendering for A* and RRT planning results.
 */
#include "motion_planner/visualizer.hpp"

namespace motion_planner {

Visualizer::Visualizer(const Grid& grid, int cell_size)
    : grid_(grid), cell_size_(cell_size),
      image_(grid.height() * cell_size, grid.width() * cell_size,
             CV_8UC3, kFreeColor) {}

// ─── Primitives ──────────────────────────────────────────────────────────────

void Visualizer::drawCell(int x, int y, cv::Scalar color) {
    const int px = x * cell_size_;
    const int py = y * cell_size_;
    cv::rectangle(image_,
        cv::Point(px, py),
        cv::Point(px + cell_size_ - 1, py + cell_size_ - 1),
        color, cv::FILLED);
}

void Visualizer::drawGrid() {
    for (int y = 0; y < grid_.height(); ++y)
        for (int x = 0; x < grid_.width(); ++x)
            if (grid_.isObstacle(x, y))
                drawCell(x, y, kObstacleColor);
}

void Visualizer::drawPath(const PlanResult& result) {
    for (const auto& p : result.path)
        drawCell(p.x, p.y, kPathColor);
}

void Visualizer::drawPoint(Point2D p, cv::Scalar color, int radius) {
    const int cx = p.x * cell_size_ + cell_size_ / 2;
    const int cy = p.y * cell_size_ + cell_size_ / 2;
    cv::circle(image_, cv::Point(cx, cy), radius, color, cv::FILLED);
}

// ─── A* Scene ────────────────────────────────────────────────────────────────

void Visualizer::renderScene(const PlanResult& result, Point2D start, Point2D goal) {
    image_.setTo(kFreeColor);
    drawGrid();
    drawPath(result);
    drawPoint(start, kStartColor);
    drawPoint(goal, kGoalColor);
}

void Visualizer::show(const std::string& title) {
    cv::imshow(title, image_);
    cv::waitKey(0);
}

void Visualizer::show(const PlanResult& result, Point2D start, Point2D goal,
                      const std::string& title) {
    renderScene(result, start, goal);
    cv::imshow(title, image_);
    cv::waitKey(0);
}

void Visualizer::save(const std::string& path, const PlanResult& result,
                      Point2D start, Point2D goal) {
    renderScene(result, start, goal);
    cv::imwrite(path, image_);
}

// ─── RRT Scene ───────────────────────────────────────────────────────────────

void Visualizer::drawRRTTree(const RRTResult& result) {
    for (const auto& [from, to] : result.tree_edges) {
        cv::Point p1(from.x * cell_size_ + cell_size_ / 2,
                     from.y * cell_size_ + cell_size_ / 2);
        cv::Point p2(to.x * cell_size_ + cell_size_ / 2,
                     to.y * cell_size_ + cell_size_ / 2);
        cv::line(image_, p1, p2, kRRTTreeColor, 1);
    }
}

void Visualizer::drawRRTPath(const RRTResult& result) {
    for (size_t i = 1; i < result.path.size(); ++i) {
        cv::Point p1(result.path[i - 1].x * cell_size_ + cell_size_ / 2,
                     result.path[i - 1].y * cell_size_ + cell_size_ / 2);
        cv::Point p2(result.path[i].x * cell_size_ + cell_size_ / 2,
                     result.path[i].y * cell_size_ + cell_size_ / 2);
        cv::line(image_, p1, p2, kRRTPathColor, 2);
    }
}

void Visualizer::renderScene(const RRTResult& result, Point2D start, Point2D goal) {
    image_.setTo(kFreeColor);
    drawGrid();
    drawRRTTree(result);
    drawRRTPath(result);
    drawPoint(start, kStartColor, 6);
    drawPoint(goal, kGoalColor, 6);
}

void Visualizer::show(const RRTResult& result, Point2D start, Point2D goal,
                      const std::string& title) {
    renderScene(result, start, goal);
    cv::imshow(title, image_);
    cv::waitKey(0);
}

void Visualizer::save(const std::string& path, const RRTResult& result,
                      Point2D start, Point2D goal) {
    renderScene(result, start, goal);
    cv::imwrite(path, image_);
}

} // namespace motion_planner
