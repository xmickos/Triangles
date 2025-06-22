#include<unordered_set>
#include "triangles.hpp"
#include<numeric>

#pragma once

namespace hw3d {

    struct AABB final {
        Vector3d min, max;

        AABB() = default;
        AABB(const Vector3d& min_, const Vector3d& max_) : min(min_), max(max_) {}
        AABB(const Triangle& tr)  {
            Vector3d min_(std::numeric_limits<double>::max()), max_(-std::numeric_limits<double>::max());

            for(auto vertex = tr.cbegin(), et = tr.cend(); vertex != et; ++vertex) {
                min_[0] = std::min((*vertex)[0], min_[0]);
                min_[1] = std::min((*vertex)[1], min_[1]);
                min_[2] = std::min((*vertex)[2], min_[2]);

                max_[0] = std::max((*vertex)[0], max_[0]);
                max_[1] = std::max((*vertex)[1], max_[1]);
                max_[2] = std::max((*vertex)[2], max_[2]);
            }

            max = max_;
            min = min_;
        }

        void update(const Triangle& tr) {
            for(auto vertex = tr.cbegin(), et = tr.cend(); vertex != et; ++vertex) {
                min[0] = std::min((*vertex)[0], min[0]);
                min[1] = std::min((*vertex)[1], min[1]);
                min[2] = std::min((*vertex)[2], min[2]);

                max[0] = std::max((*vertex)[0], max[0]);
                max[1] = std::max((*vertex)[1], max[1]);
                max[2] = std::max((*vertex)[2], max[2]);
            }
        }

        bool is_inside(const AABB& outer) { // "this" is inside "outer"
            return std::equal(min.cbegin(), min.cend(), outer.min.cbegin(),
                [](double x1, double x2) { return x1 > x2; }
                ) && std::equal(max.cbegin(), max.cend(), outer.max.cbegin(),
                [](double x1, double x2) { return x1 < x2; }
            );
        }

        std::vector<AABB> subdivide() const {
            std::vector<Vector3d> vertexes(8);
            std::vector<AABB> aabbs(8);
            Vector3d middle_point((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2);

            /*
                   6 --------- 7
                 / |         / |
                5 --------- 4  |
                |  |        |  |
                |  2 -------|--3
                | /         | /
                1 --------- 0
            */

            vertexes[0] = min;
            vertexes[1] = Vector3d(max[0], min[1], min[2]);
            vertexes[2] = Vector3d(max[0], max[1], min[2]);
            vertexes[3] = Vector3d(min[0], max[1], min[2]);
            vertexes[4] = Vector3d(min[0], min[1], max[2]);
            vertexes[5] = Vector3d(max[0], max[1], min[2]);
            vertexes[6] = max;
            vertexes[7] = Vector3d(max[0], min[1], max[2]);

            std::transform(vertexes.begin(), vertexes.end(), aabbs.begin(),
                [&](const Vector3d& vertex){ return AABB(middle_point, vertex); }
            );

            return aabbs;
        }

        long double volume() const noexcept { return std::fabs((max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2])); }
        long double diag_len() const noexcept { return std::sqrt((max[0] - min[0])*(max[0] - min[0]) + (max[1] - min[1])*(max[1] - min[1]) + (max[2] - min[2])*(max[2] - min[2])); }

    };

    class Octree final {
        private:
            struct Node {
                AABB bounds;
                std::vector<size_t> triangle_indices;
                std::vector<std::unique_ptr<Node>> children;

                Node(AABB bounds_) : bounds(bounds_) {}
                Node() = default;

                void subdivide(int curr_depth, int max_depth) {
                    if(curr_depth == max_depth) { return; }

                    std::vector<AABB> aabbs = bounds.subdivide();
                    for(auto&& aabb : aabbs) {
                        children.push_back(std::make_unique<Node>(aabb));
                    }

                    for(auto&& child : children) {
                        child->subdivide(curr_depth + 1, max_depth);
                    }
                }

                double volume() const noexcept {
                    std::vector<double> dxdydz(3);
                    std::transform(bounds.min.cbegin(), bounds.min.cend(), bounds.max.cbegin(), dxdydz.begin(),
                        [](double x1, double x2){ return std::fabs(x1 - x2); }
                    );
                    return std::accumulate(dxdydz.begin(), dxdydz.end(), 1.0, std::multiplies<double>());
                }
            };

            Node root;
            double eps = 1e-5;

        public:
            Octree(AABB rootBounds, int max_depth = 10) : root(rootBounds) {
                root.subdivide(0, max_depth);
            }

            void insert(const Triangle& tr, size_t idx) {
                AABB bb(tr);
                #if 0
                    size_t depth = std::floor(std::log2f(root.bounds.volume() / bb.volume()) / 3.0);    // bad idea
                    // V_n = V_0 / 8^n => n = std::floor(1/3 * log_2(V_0 / V_n))
                #endif

                size_t depth =  std::floor(std::log2(root.bounds.diag_len() / (root.bounds.diag_len() + eps)));
                // diag_0 / diag_n = 2^n => n = std::floor(log2(diag_0 / diag_n));

                Node* curr_node = &root;
                size_t curr_depth = 0;
                while(curr_depth <= depth) {
                    auto it = std::find_if(curr_node->children.begin(), curr_node->children.end(),
                        [&](const std::unique_ptr<Node>& child){ return bb.is_inside(child.get()->bounds); }
                    );
                    if(it == curr_node->children.end()) {
                        curr_node->triangle_indices.push_back(idx);
                        return;
                    } else {
                        curr_node = it->get();
                        curr_depth++;
                    }
                }
                std::cout << "Failed to insert triangle " << tr << " in the tree :(" << std::endl;
                std::terminate();
            }

        private:

            void start_discovering(const Node& curr_node, const std::vector<Triangle>& triangles, std::unordered_set<size_t>& output_idxs) const {
                if(!curr_node.triangle_indices.empty()) {
                    for(size_t i = 0; i < curr_node.triangle_indices.size(); ++i) {
                        for(size_t j = 0; j < i; ++j) {
                            if(intersection_test_3d(triangles[curr_node.triangle_indices[i]], triangles[curr_node.triangle_indices[j]])) {
                                output_idxs.insert(i);
                                output_idxs.insert(j);
                            }
                        }
                    }
                    downstream_counting(curr_node, triangles, output_idxs);
                }
                for(const auto& child : curr_node.children) {
                    start_discovering(*child, triangles, output_idxs);  // could be bad
                }
            }

            void downstream_counting(const Node& curr_node, const std::vector<Triangle>& triangles, std::unordered_set<size_t>& output_idxs) const {
                for(auto&& t_idx : curr_node.triangle_indices) {
                    for(auto&& child : curr_node.children) {
                        for(auto&& child_t_idx : child->triangle_indices) {
                            if(intersection_test_3d(triangles[t_idx], triangles[child_t_idx])) {
                                output_idxs.insert(t_idx);
                                output_idxs.insert(child_t_idx);
                            }
                        }
                    }
                }
            }

        private:

            void dump_(const Node& curr_node) const {
                if(!curr_node.triangle_indices.empty()) {
                    std::cout << "(";
                    std::for_each(curr_node.triangle_indices.begin(), std::next(curr_node.triangle_indices.end(), -1),
                        [](auto idx){ std::cout << idx << " ";}
                    );
                    std::cout << curr_node.triangle_indices.back() << ")";
                }
                for(const auto& child : curr_node.children) {
                    dump_(*child);  // could be bad
                }
            }

        public:

            void dump() const { dump_(root); }

            std::unordered_set<size_t> count_intersections(std::vector<Triangle>& vec) const {
                std::unordered_set<size_t> output_idxs;
                start_discovering(root, vec, output_idxs);
                return output_idxs;
            }
    };

}
