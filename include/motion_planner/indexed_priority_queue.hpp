/**
 * @file indexed_priority_queue.hpp
 * @brief Min-heap with O(log n) decreaseKey for A*.
 *
 * Why not std::priority_queue?
 * std::priority_queue lacks decreaseKey, so the common workaround is to push
 * duplicate entries and skip stale ones on pop — giving O(E log E) complexity.
 * This IPQ supports decreaseKey natively, achieving true O(E log V).
 *
 * Uses Structure-of-Arrays (SoA) layout: three parallel vectors instead of a
 * single vector of structs. This keeps the heap indices (ints) packed together
 * for tighter loops during sift operations.
 */
#pragma once

#include <algorithm>
#include <vector>

namespace motion_planner {

/**
 * @class IndexedPriorityQueue
 * @brief Min-heap mapping grid indices to f-scores with O(log n) decreaseKey.
 *
 * Internal layout (SoA):
 *   - `heap_[pos]`          → grid_index at heap position `pos`
 *   - `index_in_heap_[idx]` → heap position of grid_index `idx` (-1 if absent)
 *   - `f_scores_[idx]`      → f-score of grid_index `idx`
 */
class IndexedPriorityQueue {
public:
    explicit IndexedPriorityQueue(int capacity)
        : index_in_heap_(capacity, -1), f_scores_(capacity, 0.0) {
        heap_.reserve(capacity);
    }

    [[nodiscard]] bool empty()    const { return heap_.empty(); }
    [[nodiscard]] int  size()     const { return static_cast<int>(heap_.size()); }
    [[nodiscard]] bool contains(int grid_index) const {
        return index_in_heap_[grid_index] != -1;
    }

    void push(int grid_index, double f_score) {
        f_scores_[grid_index] = f_score;
        heap_.push_back(grid_index);
        index_in_heap_[grid_index] = static_cast<int>(heap_.size()) - 1;
        siftUp(static_cast<int>(heap_.size()) - 1);
    }

    /// Remove and return the element with the smallest f-score.
    int pop() {
        int top = heap_[0];
        index_in_heap_[top] = -1;

        heap_[0] = heap_.back();
        heap_.pop_back();

        if (!heap_.empty()) {
            index_in_heap_[heap_[0]] = 0;
            siftDown(0);
        }
        return top;
    }

    /// Update an existing entry to a lower f-score and restore heap order.
    void decreaseKey(int grid_index, double new_f_score) {
        f_scores_[grid_index] = new_f_score;
        siftUp(index_in_heap_[grid_index]);
    }

private:
    std::vector<int>    index_in_heap_;  ///< grid_index → heap position (-1 = absent)
    std::vector<double> f_scores_;       ///< grid_index → f-score
    std::vector<int>    heap_;           ///< heap[pos]  → grid_index

    void siftUp(int i) {
        while (i > 0) {
            int parent = (i - 1) / 2;
            if (f_scores_[heap_[i]] < f_scores_[heap_[parent]]) {
                std::swap(heap_[i], heap_[parent]);
                index_in_heap_[heap_[i]]      = i;
                index_in_heap_[heap_[parent]]  = parent;
                i = parent;
            } else {
                break;
            }
        }
    }

    void siftDown(int i) {
        int n = static_cast<int>(heap_.size());
        while (true) {
            int left     = 2 * i + 1;
            int right    = 2 * i + 2;
            int smallest = i;

            if (left < n && f_scores_[heap_[left]] < f_scores_[heap_[smallest]])
                smallest = left;
            if (right < n && f_scores_[heap_[right]] < f_scores_[heap_[smallest]])
                smallest = right;

            if (smallest != i) {
                std::swap(heap_[i], heap_[smallest]);
                index_in_heap_[heap_[i]]        = i;
                index_in_heap_[heap_[smallest]]  = smallest;
                i = smallest;
            } else {
                break;
            }
        }
    }
};

} // namespace motion_planner