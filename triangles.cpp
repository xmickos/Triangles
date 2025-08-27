#include <iostream>
#include "include/bvh.hpp"
#include "include/generate_tests.hpp"

using namespace hw3d;

int main() {

    size_t N;
    double x1, y1, z1, x2, y2, z2, x3, y3, z3;

    std::vector<Triangle> triangles;
    triangles.reserve(N);
    AABB scene_bb;

    std::cin >> N;

    for(int i = 0; i < N; ++i) {
        std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;

        scene_bb.expand_triangle_raw(x1, y1, z1, x2, y2, z2, x3, y3, z3);
        triangles.emplace_back(Vector3d{x1, y1, z1}, Vector3d{x2, y2, z2}, Vector3d{x3, y3, z3});
    }

    Octree scene_tree(scene_bb);

    for(int i = 0; i < triangles.size(); ++i) {
        scene_tree.insert(triangles[i], i);
    }

    std::unordered_set<size_t> indexes = scene_tree.count_intersections(triangles);
    std::cout << indexes.size() << std::endl;

    return 0;
}
