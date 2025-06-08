#include<iostream>
#include"include/bvh.hpp"

using namespace hw3d;

int main() {

    Triangle T0(Vector3d(0, 0, 0), Vector3d(2, 0, 0), Vector3d(4, 0, 0));

    // считали в vec и в scene_bb и в max_depth
    std::vector<Triangle> triangles;
    AABB scene_bb(T0);
    size_t max_depth;

    Octree scene_tree(scene_bb, max_depth);

    for(auto&& triangle : triangles) {
        scene_tree.insert(triangle);
    }

    auto set = scene_tree.count_intersections(triangles);
    std::for_each(set.begin(), set.end(), [](auto i){ std::cout << i << " "; });
    return 0;
}
