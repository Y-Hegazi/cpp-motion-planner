/**
 * @file grid.hpp
 * @brief 2D occupancy grid with obstacle management and neighbor queries.
 *
 * Performance note: the grid is stored as a flat 1D vector (row-major) instead
 * of a 2D vector-of-vectors. This improves cache locality because all cells
 * are contiguous in memory, which matters when A* scans thousands of cells.
 */
#pragma once

#include <cstdint>
#include <random>
#include <vector>

namespace motion_planner {

/// Simple 2D integer point used throughout the planner.
struct Point2D {
    int x;
    int y;
};

/// Semantic cell types for visualization and debugging.
enum class CellType : std::uint8_t {
    FREE     = 0,
    OBSTACLE = 1,
    START    = 2,
    GOAL     = 3,
    PATH     = 4,
    VISITED  = 5
};

/**
 * @class Grid
 * @brief Manages a width × height occupancy grid.
 *
 * Internally uses a **flat 1D `std::vector<CellType>`** indexed as
 * `cells_[y * width + x]` for optimal cache performance. Public accessors
 * convert between (x, y) coordinates and flat indices.
 */
class Grid {
public:
    Grid(int width, int height);

    void setObstacle(int x, int y);
    void clearCell(int x, int y);

    [[nodiscard]] bool isObstacle(int x, int y) const;
    [[nodiscard]] bool isValid(int x, int y)    const;

    /// @brief Returns free neighbors (4- or 8-connected).
    [[nodiscard]] std::vector<Point2D> getFreeNeighbors(
        int x, int y, bool eight_connected = false) const;

    /// @brief Fills a rectangular region with obstacles.
    void addWall(int x0, int y0, int x1, int y1);

    /// @brief Randomly sets cells as obstacles with the given density ∈ [0, 1].
    void randomize(double obstacle_density, unsigned seed = 0);

    void print() const;

    [[nodiscard]] int    width()  const { return width_; }
    [[nodiscard]] int    height() const { return height_; }
    [[nodiscard]] int    getIndex(int x, int y)   const { return y * width_ + x; }
    [[nodiscard]] Point2D getCoords(int index)    const { return {index % width_, index / width_}; }

private:
    int width_;
    int height_;
    std::vector<CellType> cells_;  ///< Flat 1D storage (row-major) for cache locality.
    std::mt19937 rng_;
};

} // namespace motion_planner