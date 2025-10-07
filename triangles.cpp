#include <iostream>
#include "include/bvh.hpp"
#include "include/generate_tests.hpp"

using namespace hw3d;

int main() {

    size_t N;
    double x1, y1, z1, x2, y2, z2, x3, y3, z3;
    std::cin >> N;

    std::vector<Triangle> triangles;
    triangles.reserve(N);

    AABB scene_bb;

    for(size_t i = 0; i < N; ++i) {
        std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;

        scene_bb.expand_triangle_raw(x1, y1, z1, x2, y2, z2, x3, y3, z3);
        triangles.emplace_back(Vector3d{x1, y1, z1}, Vector3d{x2, y2, z2}, Vector3d{x3, y3, z3});
    }

    Octree scene_tree(scene_bb);

    for(size_t i = 0; i < triangles.size(); ++i) {
        scene_tree.insert(triangles[i], i);         // TODO add padding for final AABB & cache AABBs for triangles
    }                                               // instead of constructing a new one in each insert

    std::vector<size_t> indexes(triangles.size() / 2);
    indexes = scene_tree.count_intersections(triangles);
    std::sort(indexes.begin(), indexes.end());
    std::for_each(indexes.begin(), indexes.end(), [](auto it){ std::cout << it << std::endl; });
    std::cout << std::endl;

    return 0;
}
