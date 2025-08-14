#include <iostream>
#include "include/bvh.hpp"
#include "include/generate_tests.hpp"

using namespace hw3d;

int main() {

    Triangle T1(Vector3d(5, 0, 1), Vector3d(0, 5, 1), Vector3d(-5, 0, 1));
    Triangle T2(Vector3d(1.5, 1.5, 0), Vector3d(0, 1.5, 0), Vector3d(1, 0, 0));
    Triangle T3(Vector3d(0.5, 0.5, 0), Vector3d(0.5, 2, 0), Vector3d(1.5, 0.5, 0));

    // считали в vec и в scene_bb и в max_depth
    Vector3d min = {0, 0, 0};
    Vector3d max = {10, 10, 10};
    std::vector<Triangle> triangles = generate_zero_intersections_test(min, max);
    // Triangle tr = Triangle({5.20426, 5.79955, 5.44044}, {5.73276, 5.65721, 5.39043}, {5.75377, 5.70372, 5.83115});
    // std::vector<Triangle> triangles = {tr};
    AABB scene_bb(min, max);
    for(auto&& tr : triangles) {
        std::cout << tr << std::endl;
    }
    // triangles.push_back(T1);
    // triangles.push_back(T2);
    // triangles.push_back(T3);
    // AABB scene_bb(T1);
    // scene_bb.update(T2);
    // scene_bb.update(T3);
    std::cout << "scene_bb: " << scene_bb.max << " and " << scene_bb.min << ", " << triangles.size() << " triangles." << std::endl;
    size_t max_depth = 2;

    Octree scene_tree(scene_bb, max_depth);

    for(int i = 0; i < triangles.size(); ++i) {
        scene_tree.insert(triangles[i], i);
    }

    scene_tree.dump();

    auto set = scene_tree.count_intersections(triangles);
    std::for_each(set.begin(), set.end(), [](auto i){ std::cout << i << " "; });
    std::cout << std::endl << "Empty? " << (set.empty() ? "Yes" : "No") << std::endl;
    return 0;
}
