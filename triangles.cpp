#include<iostream>
#include"include/bvh.hpp"

using namespace hw3d;

int main() {

    Triangle T1(Vector3d(-1, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, 0));
    Triangle T2(Vector3d(-2, 0, 0), Vector3d(0, 2, 0), Vector3d(2, 0, 0));
    Triangle T3(Vector3d(-2, 0, 1), Vector3d(0, 2, 1), Vector3d(2, 0, 1));

    // считали в vec и в scene_bb и в max_depth
    std::vector<Triangle> triangles;
    triangles.push_back(T1);
    triangles.push_back(T2);
    triangles.push_back(T3);
    AABB scene_bb(T1);
    scene_bb.update(T2);
    scene_bb.update(T3);
    size_t max_depth = 2;

    Octree scene_tree(scene_bb, max_depth);

    for(int i = 0; i < triangles.size(); ++i) {
        scene_tree.insert(triangles[i], i);
    }

    std::cout << "Success:)" << std::endl;

    // auto set = scene_tree.count_intersections(triangles);
    // std::for_each(set.begin(), set.end(), [](auto i){ std::cout << i << " "; });
    return 0;
}
