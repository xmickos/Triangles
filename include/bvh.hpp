#include<unordered_set>
#include "triangles.hpp"

#pragma once

namespace hw3d {

    struct AABB final {
        Vector3d min, max;

        AABB();

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

        AABB decrease(int i) const noexcept {
            return
        }

        long double volume() const noexcept { return std::fabs((max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2])); }

    };

    class Octree final {
        private:
            struct Node {
                AABB bounds;
                std::vector<Node> children(8);

                std::vector<Node> subdivide() const {
                    std::vector<Node> children_(8);

                    children_[0] = Node(bounds.decrease())
                }
            };

            Node root;

        public:
            Node* build_octree(Node* curr_node, size_t curr_depth, size_t max_depth) {
                while(curr_depth < max_depth) {
                    for(auto&& child : curr_node->children) {
                        child.children = child.subdivide();
                    }
                }
            }

            Octree(AABB rootBounds, int max_depth = 10) : root(rootBounds, nullptr, nullptr) {
                build_octree(root, 0, max_depth);
            }

            void insert(const Triangle& tr);

            std::unordered_set<size_t> count_intersections(std::vector<Triangle>& vec) const {

            }
    };

}
