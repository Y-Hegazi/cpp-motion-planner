#pragma once

#include <vector>
#include <algorithm>

namespace motion_planner {

class IndexedPriorityQueue {
public:
    explicit IndexedPriorityQueue(int size)
        : index_in_heap_(size, -1), f_scores_(size, 0.0) {
        heap_.reserve(size);
    }

    [[nodiscard]] bool empty() const { return heap_.empty(); }
    [[nodiscard]] int  size()  const { return static_cast<int>(heap_.size()); }
    [[nodiscard]] bool contains(int grid_index) const { return index_in_heap_[grid_index] != -1; }

    void push(int grid_index, double f_score) {
        f_scores_[grid_index] = f_score;
        heap_.push_back(grid_index);
        index_in_heap_[grid_index] = static_cast<int>(heap_.size()) - 1;
        siftUp(static_cast<int>(heap_.size()) - 1);
    }

    // Remove and return the element with the smallest f-score.
    int pop() {
        int top = heap_[0];
        index_in_heap_[top] = -1;

        // Move last element to root
        heap_[0] = heap_.back();
        heap_.pop_back();

        if (!heap_.empty()) {
            index_in_heap_[heap_[0]] = 0;
            siftDown(0);
        }
        return top;
    }

    void decreaseKey(int grid_index, double new_f_score) {
        f_scores_[grid_index] = new_f_score;
        siftUp(index_in_heap_[grid_index]);
    }

private:
    std::vector<int>    index_in_heap_;  // grid_index → position in heap (-1 = absent)
    std::vector<double> f_scores_;       // grid_index → f-score
    std::vector<int>    heap_;           // heap[position] → grid_index

    void siftUp(int i) {
        while (i > 0) {
            int parent = (i - 1) / 2;
            if (f_scores_[heap_[i]] < f_scores_[heap_[parent]]) {
                std::swap(heap_[i], heap_[parent]);
                index_in_heap_[heap_[i]] = i;
                index_in_heap_[heap_[parent]] = parent;
                i = parent;
            } else {
                break;
            }
        }
    }

    void siftDown(int i) {
        int n = static_cast<int>(heap_.size());
        while (true) {
            int left  = 2 * i + 1;
            int right = 2 * i + 2;
            int smallest = i;

            if (left < n && f_scores_[heap_[left]] < f_scores_[heap_[smallest]])
                smallest = left;
            if (right < n && f_scores_[heap_[right]] < f_scores_[heap_[smallest]])
                smallest = right;

            if (smallest != i) {
                std::swap(heap_[i], heap_[smallest]);
                index_in_heap_[heap_[i]] = i;
                index_in_heap_[heap_[smallest]] = smallest;
                i = smallest;
            } else {
                break;
            }
        }
    }
};

}  