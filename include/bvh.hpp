#include<unordered_set>
#include "triangles.hpp"

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

        bool is_inside(const AABB& outer) {
            return std::all_of(min.cbegin(), min.cend(), ) // TODO
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
                    return std::accumulate(dxdydz.begin(), dxdydz.end(), 1f, std::multiplies<double>());
                }
            };

            Node root;

        public:
            Octree(AABB rootBounds, int max_depth = 10) : root(rootBounds) {
                root.subdivide(0, max_depth);
            }

            void insert(const Triangle& tr) {
                AABB bb(tr);
                size_t depth = std::floor(std::log2f(root.bounds.volume() / bb.volume()) / 3f);
                // V_n = V_0 / 8^n => n = std::floor(1/3 * log_2(V_0 / V_n))

                Node* curr_node = &root;
                size_t curr_depth = 0;
                while(curr_node.bounds.is_inside(bb) && curr_depth < depth) {
                    go_down(); // TODO
                }
            }

            std::unordered_set<size_t> count_intersections(std::vector<Triangle>& vec) const {}
    };

}
