// MIT License

// Copyright (c) 2017 gishi523

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Modified work [2021] Flux Auto

/**
 * @file kdtree.h
 * @author Path Planning (shivam.nagar@fluxauto.xyz)
 * @brief header file for kdtree data structure.
 * @version 1.0
 * @date 21/02/2021
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef HOME_ROCKET_ASGARD_SRC_PATH_PLANNER_INCLUDE_BOPT_WAREHOUSE_KDTREE_H_
#define HOME_ROCKET_ASGARD_SRC_PATH_PLANNER_INCLUDE_BOPT_WAREHOUSE_KDTREE_H_

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

namespace KDT {

class MyPoint : public std::array<double, 2> {
 public:
    // dimension of space (or "k" of k-d tree)
    // KDTree class accesses this member
    static const int DIM = 2;
    float cost;
    // the constructors
    MyPoint() {}
    MyPoint(double x, double y) {
        (*this)[0] = x;
        (*this)[1] = y;
    }
    MyPoint(double x, double y, float cost_) : cost(cost_) {
        (*this)[0] = x;
        (*this)[1] = y;
    }
};
/** @brief k-d tree class.
 */
template <class PointT>
class KDTree {
 public:
    /** @brief The constructors.
     */
    KDTree() : root_(nullptr) {}
    explicit KDTree(const std::vector<PointT>& points) : root_(nullptr) {
        build(points);
    }

    /** @brief The destructor.
     */
    ~KDTree() { clear(); }

    /** @brief Re-builds k-d tree.
     */
    void build(const std::vector<PointT>& points) {
        clear();

        points_ = points;

        std::vector<int> indices(points.size());
        std::iota(std::begin(indices), std::end(indices), 0);

        root_ =
            buildRecursive(indices.data(), static_cast<int>(points.size()), 0);
    }

    /** @brief Clears k-d tree.
     */
    void clear() {
        clearRecursive(root_);
        root_ = nullptr;
        points_.clear();
    }

    /** @brief Validates k-d tree.
     */
    bool validate() const {
        try {
            validateRecursive(root_, 0);
        } catch (const Exception&) {
            return false;
        }

        return true;
    }

    /** @brief Searches the nearest neighbor.
     */
    int nnSearch(const PointT& query, double* minDist = nullptr) const {
        int guess;
        double _minDist = std::numeric_limits<double>::max();

        nnSearchRecursive(query, root_, &guess, &_minDist);

        if (minDist) *minDist = _minDist;

        return guess;
    }

    /** @brief Searches k-nearest neighbors.
     */
    std::vector<int> knnSearch(const PointT& query, int k) const {
        std::shared_ptr<KnnQueue> queue = std::make_shared<KnnQueue>(k);
        knnSearchRecursive(query, root_, queue, k);

        std::vector<int> indices(queue.size());
        for (size_t i = 0; i < queue.size(); i++) indices[i] = queue[i].second;

        return indices;
    }

    /** @brief Searches neighbors within radius.
     */
    std::vector<int> radiusSearch(const PointT& query, double radius) const {
        std::shared_ptr<std::vector<int>> indices =
            std::make_shared<std::vector<int>>();
        radiusSearchRecursive(query, root_, indices, radius);
        return *indices;
    }

 private:
    /** @brief k-d tree node.
     */
    struct Node {
        int idx;        //!< index to the original point
        Node* next[2];  //!< pointers to the child nodes
        int axis;       //!< dimension's axis

        Node() : idx(-1), axis(-1) { next[0] = next[1] = nullptr; }
    };

    /** @brief k-d tree exception.
     */
    class Exception : public std::exception {
        using std::exception::exception;
    };

    /** @brief Bounded priority queue.
     */
    template <class T, class Compare = std::less<T>>
    class BoundedPriorityQueue {
     public:
        BoundedPriorityQueue() = delete;
        explicit BoundedPriorityQueue(size_t bound) : bound_(bound) {
            elements_.reserve(bound + 1);
        }

        void push(const T& val) {
            auto it = std::find_if(
                std::begin(elements_), std::end(elements_),
                [&](const T& element) { return Compare()(val, element); });
            elements_.insert(it, val);

            if (elements_.size() > bound_) elements_.resize(bound_);
        }

        const T& back() const { return elements_.back(); }
        const T& operator[](size_t index) const { return elements_[index]; }
        size_t size() const { return elements_.size(); }

     private:
        size_t bound_;
        std::vector<T> elements_;
    };

    /** @brief Priority queue of <distance, index> pair.
     */
    using KnnQueue = BoundedPriorityQueue<std::pair<double, int>>;

    /** @brief Builds k-d tree recursively.
     */
    Node* buildRecursive(int* indices, int npoints, int depth) {
        if (npoints <= 0) return nullptr;

        const int axis = depth % PointT::DIM;
        const int mid = (npoints - 1) / 2;

        std::nth_element(indices, indices + mid, indices + npoints,
                         [&](int lhs, int rhs) {
                             return points_[lhs][axis] < points_[rhs][axis];
                         });

        Node* node = new Node();
        node->idx = indices[mid];
        node->axis = axis;

        node->next[0] = buildRecursive(indices, mid, depth + 1);
        node->next[1] =
            buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

        return node;
    }

    /** @brief Clears k-d tree recursively.
     */
    void clearRecursive(Node* node) {
        if (node == nullptr) return;

        if (node->next[0]) clearRecursive(node->next[0]);

        if (node->next[1]) clearRecursive(node->next[1]);

        delete node;
    }

    /** @brief Validates k-d tree recursively.
     */
    void validateRecursive(const Node* node, int depth) const {
        if (node == nullptr) return;

        const int axis = node->axis;
        const Node* node0 = node->next[0];
        const Node* node1 = node->next[1];

        if (node0 && node1) {
            if (points_[node->idx][axis] < points_[node0->idx][axis])
                throw Exception();

            if (points_[node->idx][axis] > points_[node1->idx][axis])
                throw Exception();
        }

        if (node0) validateRecursive(node0, depth + 1);

        if (node1) validateRecursive(node1, depth + 1);
    }

    static double distance(const PointT& p, const PointT& q) {
        double dist = 0;
        for (size_t i = 0; i < PointT::DIM; i++)
            dist += (p[i] - q[i]) * (p[i] - q[i]);
        return sqrt(dist);
    }

    /** @brief Searches the nearest neighbor recursively.
     */
    void nnSearchRecursive(const PointT& query, const Node* node, int* guess,
                           double* minDist) const {
        if (node == nullptr) return;

        const PointT& train = points_[node->idx];

        const double dist = distance(query, train);
        if (dist < *minDist) {
            *minDist = dist;
            *guess = node->idx;
        }

        const int axis = node->axis;
        const int dir = query[axis] < train[axis] ? 0 : 1;
        nnSearchRecursive(query, node->next[dir], guess, minDist);

        const double diff = fabs(query[axis] - train[axis]);
        if (diff < *minDist)
            nnSearchRecursive(query, node->next[!dir], guess, minDist);
    }

    /** @brief Searches k-nearest neighbors recursively.
     */
    void knnSearchRecursive(const PointT& query, const Node* node,
                            std::shared_ptr<KnnQueue> queue, int k) const {
        if (node == nullptr) return;

        const PointT& train = points_[node->idx];

        const double dist = distance(query, train);
        queue->push(std::make_pair(dist, node->idx));

        const int axis = node->axis;
        const int dir = query[axis] < train[axis] ? 0 : 1;
        knnSearchRecursive(query, node->next[dir], queue, k);

        const double diff = fabs(query[axis] - train[axis]);
        if (static_cast<int>(queue->size()) < k || diff < queue->back().first)
            knnSearchRecursive(query, node->next[!dir], queue, k);
    }

    /** @brief Searches neighbors within radius.
     */
    void radiusSearchRecursive(const PointT& query, const Node* node,
                               std::shared_ptr<std::vector<int>> indices,
                               double radius) const {
        if (node == nullptr) return;

        const PointT& train = points_[node->idx];

        const double dist = distance(query, train);
        if (dist < radius) indices->push_back(node->idx);

        const int axis = node->axis;
        const int dir = query[axis] < train[axis] ? 0 : 1;
        radiusSearchRecursive(query, node->next[dir], indices, radius);

        const double diff = fabs(query[axis] - train[axis]);
        if (diff < radius)
            radiusSearchRecursive(query, node->next[!dir], indices, radius);
    }

    Node* root_;                  //!< root node
    std::vector<PointT> points_;  //!< points
};
}  // namespace KDT

#endif  // HOME_ROCKET_ASGARD_SRC_PATH_PLANNER_INCLUDE_BOPT_WAREHOUSE_KDTREE_H_
