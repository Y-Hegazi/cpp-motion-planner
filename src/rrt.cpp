/**
 * @file rrt.cpp
 * @brief RRT implementation with a dynamic KD-Tree for O(log n) nearest-neighbor.
 *
 * Data structures (all flat-vector based for cache efficiency):
 *   - RRTNode: {x, y, parent_index} stored in a contiguous std::vector.
 *   - KDTree:  dynamic 2D spatial index stored as a flat vector of {node_idx, left, right}.
 *
 * Both structures use integer indices instead of pointers, which avoids
 * invalidation when the vectors grow and keeps memory accesses predictable.
 */
#include "motion_planner/rrt.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace {

/// Internal tree node — kept out of the public API.
struct RRTNode {
    double x, y;
    int parent;  ///< Index into the nodes vector (-1 = tree root).
};

/// Squared Euclidean distance — avoids sqrt when only comparisons are needed.
inline double distSq(double x0, double y0, double x1, double y1) {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    return dx * dx + dy * dy;
}

/**
 * @class KDTree
 * @brief Dynamic 2D KD-Tree for O(log n) nearest-neighbor queries.
 *
 * Stores indices into the external `nodes` vector rather than copies. The
 * tree alternates splitting on X (even depth) and Y (odd depth). Nearest
 * neighbor search prunes branches that cannot contain a closer point.
 */
class KDTree {
public:
    explicit KDTree(const std::vector<RRTNode>& nodes) : nodes_(nodes) {
        kd_nodes_.reserve(5000);
    }

    void insert(int node_idx) {
        if (kd_nodes_.empty()) {
            kd_nodes_.push_back({node_idx, -1, -1});
            return;
        }
        insertAt(0, node_idx, 0);
    }

    [[nodiscard]] int findNearest(double tx, double ty) const {
        int best = -1;
        double best_sq = std::numeric_limits<double>::max();
        if (!kd_nodes_.empty())
            searchNearest(0, tx, ty, 0, best, best_sq);
        return best;
    }

private:
    struct KDNode { int node_idx, left, right; };

    const std::vector<RRTNode>& nodes_;
    std::vector<KDNode> kd_nodes_;

    void insertAt(int ki, int ni, int depth) {
        const bool go_left = (depth % 2 == 0)
            ? (nodes_[ni].x < nodes_[kd_nodes_[ki].node_idx].x)
            : (nodes_[ni].y < nodes_[kd_nodes_[ki].node_idx].y);

        int& child = go_left ? kd_nodes_[ki].left : kd_nodes_[ki].right;
        if (child == -1) {
            child = static_cast<int>(kd_nodes_.size());
            kd_nodes_.push_back({ni, -1, -1});
        } else {
            insertAt(child, ni, depth + 1);
        }
    }

    void searchNearest(int ki, double tx, double ty, int depth,
                       int& best, double& best_sq) const {
        if (ki == -1) return;

        const auto& n = nodes_[kd_nodes_[ki].node_idx];
        const double d = distSq(n.x, n.y, tx, ty);
        if (d < best_sq) { best_sq = d; best = kd_nodes_[ki].node_idx; }

        // Split on X at even depth, Y at odd depth
        const double diff = (depth % 2 == 0) ? (tx - n.x) : (ty - n.y);
        const int near  = diff < 0 ? kd_nodes_[ki].left  : kd_nodes_[ki].right;
        const int far   = diff < 0 ? kd_nodes_[ki].right : kd_nodes_[ki].left;

        searchNearest(near, tx, ty, depth + 1, best, best_sq);

        // Only visit the far subtree if it could contain a closer point
        if (diff * diff < best_sq)
            searchNearest(far, tx, ty, depth + 1, best, best_sq);
    }
};

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────

namespace motion_planner {

RRT::RRT(const Grid& grid, RRTConfig config)
    : grid_(grid), config_(config) {}

RRTResult RRT::plan(Point2D start, Point2D goal, unsigned seed) const {
    RRTResult result;

    if (!grid_.isValid(start.x, start.y) || !grid_.isValid(goal.x, goal.y))
        return result;
    if (grid_.isObstacle(start.x, start.y) || grid_.isObstacle(goal.x, goal.y))
        return result;

    // Seed RNG: 0 = non-deterministic, >0 = reproducible
    rng_.seed(seed == 0 ? std::random_device{}() : seed);
    std::uniform_real_distribution<double> rand_x(0.0, grid_.width()  - 1.0);
    std::uniform_real_distribution<double> rand_y(0.0, grid_.height() - 1.0);
    std::uniform_real_distribution<double> rand_01(0.0, 1.0);

    // Tree stored as a flat vector of nodes (contiguous memory)
    std::vector<RRTNode> nodes;
    nodes.reserve(config_.max_iterations);
    nodes.push_back({static_cast<double>(start.x),
                     static_cast<double>(start.y), -1});

    KDTree kd(nodes);
    kd.insert(0);
    result.tree_edges.reserve(config_.max_iterations);

    const double gx = goal.x, gy = goal.y;
    const double thresh_sq = config_.goal_threshold * config_.goal_threshold;
    const double step_sq   = config_.step_size * config_.step_size;

    for (int iter = 0; iter < config_.max_iterations; ++iter) {

        // 1. Sample — with goal_bias probability, sample the goal directly
        double sx, sy;
        if (rand_01(rng_) < config_.goal_bias) {
            sx = gx; sy = gy;
        } else {
            sx = rand_x(rng_); sy = rand_y(rng_);
        }

        // 2. Nearest neighbor via KD-Tree — O(log n) vs brute-force O(n)
        const int ni = kd.findNearest(sx, sy);
        const auto& nearest = nodes[ni];
        const double nd_sq = distSq(nearest.x, nearest.y, sx, sy);

        // 3. Steer — extend at most step_size toward the sample
        double nx, ny;
        if (nd_sq > step_sq) {
            const double d = std::sqrt(nd_sq);
            nx = nearest.x + ((sx - nearest.x) / d) * config_.step_size;
            ny = nearest.y + ((sy - nearest.y) / d) * config_.step_size;
        } else {
            nx = sx; ny = sy;
        }
        nx = std::clamp(nx, 0.0, static_cast<double>(grid_.width()  - 1));
        ny = std::clamp(ny, 0.0, static_cast<double>(grid_.height() - 1));

        // 4. Collision check along the edge
        if (!isEdgeFree(nearest.x, nearest.y, nx, ny))
            continue;

        // 5. Extend tree
        const int new_idx = static_cast<int>(nodes.size());
        nodes.push_back({nx, ny, ni});
        kd.insert(new_idx);
        result.tree_edges.emplace_back(
            Point2D{static_cast<int>(nearest.x), static_cast<int>(nearest.y)},
            Point2D{static_cast<int>(nx), static_cast<int>(ny)});

        // 6. Goal check
        if (distSq(nx, ny, gx, gy) <= thresh_sq) {
            result.success = true;

            std::vector<Point2D> path;
            for (int c = new_idx; c != -1; c = nodes[c].parent)
                path.push_back({static_cast<int>(std::round(nodes[c].x)),
                                static_cast<int>(std::round(nodes[c].y))});
            std::reverse(path.begin(), path.end());
            path.push_back({goal.x, goal.y});  // Snap to exact goal
            result.path = std::move(path);
            return result;
        }
    }

    return result;  // Max iterations reached — no path found
}

bool RRT::isEdgeFree(double x0, double y0, double x1, double y1) const {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 1e-6) {
        const int ix = static_cast<int>(std::round(x0));
        const int iy = static_cast<int>(std::round(y0));
        return grid_.isValid(ix, iy) && !grid_.isObstacle(ix, iy);
    }

    // Check every 0.5 cells — ensures 1-cell-wide obstacles are never skipped
    const int steps = static_cast<int>(std::ceil(dist / 0.5));
    const double sx = dx / steps;
    const double sy = dy / steps;

    for (int i = 0; i <= steps; ++i) {
        const int ix = static_cast<int>(std::round(x0 + i * sx));
        const int iy = static_cast<int>(std::round(y0 + i * sy));
        if (!grid_.isValid(ix, iy) || grid_.isObstacle(ix, iy))
            return false;
    }
    return true;
}

} // namespace motion_planner